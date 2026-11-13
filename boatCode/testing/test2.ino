/*
  RFS2027 rocket-deployed autonomous boat - firmware v2.2  (2026-11-11)
  Arduino Nano + GY-NEO6MV2 GPS + GY-271 (QMC5883P) compass + microSD + L298N (2 motors, differential)

  MISSION SEQUENCE (motors are OFF until the boat has flown and landed):
    PAD      powered on in the rocket: idles, logs, compass calibration allowed (double-reset)
    FLIGHT   launch detected (GPS altitude >= 50 m above pad, or pull-pin on D2 removed)
    LANDED   landing detected (still for 10 s after a real descent); waits 60 s for the
             water-soluble string to release and for the boat to float upright
    SELFCAL  only if the compass was never calibrated: spins in place for 60 s and calibrates itself
    NAVIGATE drives to WAYPOINTS[] with differential thrust
    ARRIVED  holds position (re-navigates if it drifts > 6 m)
  A new numbered log is written every power-up (LOG00001.CSV, LOG00002.CSV, ...).
  If the Nano resets during the mission it resumes (launch record valid for 3 hours).

  ===================== NANO "L" LED (pin 13) CODES =====================
   3 quick flashes ....... firmware started
   Every 2 s, N blinks:    1 = on pad, waiting for GPS
                           2 = on pad, GPS OK - READY FOR LAUNCH
                           4 = in flight / landed, waiting to start
                           3 = navigating
   Slow blink (1 per s) .. calibrating compass
   LED ON solid .......... arrived / holding position
   Rapid flicker at start of each 2 s cycle = WARNING: compass not working, SD card not
                           logging, or (on the pad) compass NOT calibrated yet

  ===================== COMPASS CALIBRATION ON THE PAD (no computer) =====================
   1. Press RESET, wait for the 3 quick flashes, press RESET again within 5 s.
   2. Slow blinking: hold the boat LEVEL and UPRIGHT, rotate it slowly CLOCKWISE (seen from above)
      through 2-3 full circles (30 s).
   3. 3 LONG flashes = saved (survives power-off)  |  10 rapid flashes = failed, try again.
   This also clears any old launch record. If you never calibrate, the boat calibrates itself
   in the water after landing. Heading direction/offset is learned from GPS while driving.

  OPTIONAL PULL-PIN: wire Nano D2 to GND through a pin/lanyard that pulls out at liftoff.
  Detected automatically: if D2 is ever grounded, pin removal = launch.

  Wiring = RFS2027 carrier PCB:
    GPS TX -> D4, GPS RX <- D3 (1k/2k divider) | compass SDA A4, SCL A5 | SD CS D10 MOSI D11 MISO D12 SCK D13
    L298N IN1 D8, IN2 D7 (LEFT motor), IN3 D6, IN4 D5 (RIGHT motor), ENA/ENB jumpers on
  Serial @115200 (optional): send L = simulate launch (bench test)
*/
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

// ======================= MISSION SETTINGS (edit these) =======================
struct Waypoint { int32_t lat_e7; int32_t lng_e7; };          // degrees x 10,000,000
const Waypoint WAYPOINTS[] = {
  { 317896250, -1064557654 },   // target from original RFS2027 (31.7896250, -106.4557654)
};
const uint8_t  NUM_WAYPOINTS    = sizeof(WAYPOINTS) / sizeof(WAYPOINTS[0]);
const float    LAUNCH_RISE_M    = 50.0;   // altitude above pad that means "launched"
const float    LANDING_DROP_M   = 30.0;   // must have descended this far from peak before "landed"
const uint32_t SETTLE_MS        = 60000;  // after landing: string dissolves, boat rights itself
const uint32_t SELFCAL_MS       = 60000;  // in-water self-calibration spin time
const uint16_t RESUME_WINDOW_MIN = 180;   // launch record older than this is ignored at boot
const float    DECLINATION_DEG  = 6.3;    // only until auto-aligned to GPS (El Paso +6.3)
const int8_t   HEADING_Y_SIGN   = -1;     // only until auto-aligned to GPS
const float    ARRIVE_RADIUS_M  = 3.0;    // NEO-6M is ~2.5 m accurate
const float    RESUME_RADIUS_M  = 6.0;
const float    MAX_HDOP         = 5.0;
const uint32_t FIX_TIMEOUT_MS   = 3000;
const float    TURN_ON_DEG      = 15.0, TURN_OFF_DEG = 6.0;
const float    COURSE_MIN_MPS   = 0.8;    // faster than this: steer by GPS course
bool           motorsEnabled    = true;
// =============================================================================

const uint8_t GPS_RX_PIN = 4, GPS_TX_PIN = 3, SD_CS = 10, LED = 13, LAUNCH_PIN = 2;
const uint8_t IN1 = 8, IN2 = 7, IN3 = 6, IN4 = 5;
const int EE_CAL = 0, EE_DBLRESET = 100, EE_FLIGHT = 110;

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
File logFile;
bool sdOK = false;
char logName[13] = "none";
struct GpsData {
  int32_t lat, lng;                      // degrees x 1e7, exact (no float rounding)
  float alt, hdop, mps, course;
  uint8_t sats, hh, mi, ss;
  uint32_t date;                         // ddmmyy
  bool fixQ, everFix, timeOK, dateOK, rmcOK, courseOK;
  uint32_t fixMs, rmcMs;
} g = {};

// ---------------- LED on D13 (shared with the SD clock line) ----------------
// Driven as plain output only while SPI is off; the SD library re-enables SPI at every card
// access, and the card ignores the clock while its CS (D10) is high.
bool ledState = false;
void led(bool on) { SPCR &= ~_BV(SPE); digitalWrite(LED, on); ledState = on; }
void flashes(uint8_t n, uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < n; i++) { led(true); delay(onMs); led(false); delay(offMs); }
}

// ---------------- events: Serial "***" lines + the log's "event" column ----------------
const __FlashStringHelper *pendingEvent = nullptr;
void event(const __FlashStringHelper *msg) { Serial.print(F("*** ")); Serial.println(msg); pendingEvent = msg; }

// ---------------- compass ----------------
uint8_t magAddr = 0;
struct MagCal { uint16_t magic; int16_t xoff, yoff; float xs, ys; uint8_t aligned; int8_t ysign; float offset;
                uint8_t calibrated; int8_t upZ; };
const uint16_t CAL_MAGIC = 0xC0A7;
MagCal cal;
int16_t mx, my, mz;
uint8_t magFails = 0;

void resetCal() { cal = { CAL_MAGIC, 0, 0, 1.0f, 1.0f, 0, HEADING_Y_SIGN, DECLINATION_DEG, 0, 1 }; }

void i2cWrite(uint8_t a, uint8_t r, uint8_t v) { Wire.beginTransmission(a); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
bool present(uint8_t a) { Wire.beginTransmission(a); return Wire.endTransmission() == 0; }

bool magInit() {
  magAddr = 0;
  if (!present(0x2C)) return false;                      // QMC5883P
  magAddr = 0x2C; i2cWrite(0x2C, 0x29, 0x06); i2cWrite(0x2C, 0x0B, 0x08); i2cWrite(0x2C, 0x0A, 0xCD);
  delay(20);
  return true;
}

bool magReadOnce() {
  if (!magAddr) return false;
  Wire.beginTransmission(magAddr);
  Wire.write(0x01);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(magAddr, (uint8_t)6) != 6) return false;
  uint8_t b[6];
  for (uint8_t i = 0; i < 6; i++) b[i] = Wire.read();
  mx = (int16_t)(b[0] | (b[1] << 8)); my = (int16_t)(b[2] | (b[3] << 8)); mz = (int16_t)(b[4] | (b[5] << 8));
  return true;
}

bool magRead() {
  for (uint8_t i = 0; i < 3; i++) if (magReadOnce()) { magFails = 0; return true; }
  if (++magFails >= 10) { magFails = 0; magInit(); }
  return false;
}

float wrap360(float a) { while (a < 0) a += 360; while (a >= 360) a -= 360; return a; }
float wrap180(float a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
float rawAngle(int8_t sign) {
  float fx = (mx - cal.xoff) * cal.xs, fy = (my - cal.yoff) * cal.ys;
  return atan2(sign * fy, fx) * 180.0 / PI;
}
float magHeading() { return wrap360(rawAngle(cal.ysign) + cal.offset); }

// learn the heading offset (declination + how the module is mounted) from the GPS course
// while driving straight; the direction sign comes from calibration / the in-water spin
float sA = 0, cA = 0;
uint8_t nAlign = 0;
void learnAlignment(float course) {
  float d = (course - rawAngle(cal.ysign)) * PI / 180.0;
  sA += sin(d); cA += cos(d);
  if (++nAlign < 60) return;                               // 30 s of straight driving
  if (sqrt(sA * sA + cA * cA) / nAlign > 0.9) {            // consistent -> save it
    cal.offset = atan2(sA, cA) * 180.0 / PI;
    cal.aligned = 1;
    EEPROM.put(EE_CAL, cal);
    event(F("Compass aligned to GPS"));
  }
  sA = cA = 0; nAlign = 0;
}

// min/max collection shared by pad calibration and in-water self-calibration
int16_t xmin, xmax, ymin, ymax; long zsum; uint16_t zn; float sweep, lastA;
void calStart() { xmin = ymin = 32767; xmax = ymax = -32768; zsum = 0; zn = 0; sweep = 0; }
void calSample() {
  xmin = min(xmin, mx); xmax = max(xmax, mx); ymin = min(ymin, my); ymax = max(ymax, my);
  float a = atan2(my - (ymax + ymin) / 2.0, mx - (xmax + xmin) / 2.0) * 180.0 / PI;
  if (zn) sweep += wrap180(a - lastA);   // how far (and which way) the field vector has turned
  lastA = a; zsum += mz; zn++;
}
bool calFinish(int8_t dir) {             // dir: +1 = rotated clockwise (from above), -1 = counter-clockwise
  float rx = (xmax - xmin) / 2.0, ry = (ymax - ymin) / 2.0;
  if (rx < 50 || ry < 50 || zn == 0 || fabs(sweep) < 300) { event(F("Calibration FAILED (rotate more)")); return false; }
  float r = (rx + ry) / 2.0;
  resetCal();
  cal.xoff = (xmax + xmin) / 2; cal.yoff = (ymax + ymin) / 2; cal.xs = r / rx; cal.ys = r / ry;
  cal.calibrated = 1;
  cal.ysign = sweep * dir > 0 ? 1 : -1;  // heading must increase when turning clockwise
  cal.upZ = zsum >= 0 ? 1 : -1;          // sign of vertical field when upright (for capsize check)
  EEPROM.put(EE_CAL, cal);
  event(F("Compass calibrated and saved"));
  return true;
}

// ---------------- motors ----------------
enum MotorCmd : uint8_t { M_STOP, M_FWD, M_LEFT, M_RIGHT };
MotorCmd motor = M_STOP;
const char MOTOR_TXT[][8] PROGMEM = { "STOPPED", "FORWARD", "LEFT", "RIGHT" };
void setMotors(uint8_t c) {              // uint8_t: Arduino auto-prototypes come before the enum
  if (!motorsEnabled) c = M_STOP;
  motor = (MotorCmd)c;
  bool left = (c == M_FWD || c == M_RIGHT), right = (c == M_FWD || c == M_LEFT);
  digitalWrite(IN1, left);  digitalWrite(IN2, LOW);
  digitalWrite(IN3, right); digitalWrite(IN4, LOW);
}

// ---------------- printing helpers ----------------
#define FS(x) ((const __FlashStringHelper *)(x))
void print2(Print &p, uint8_t v) { if (v < 10) p.print('0'); p.print(v); }
void printE7(Print &p, int32_t v) {
  if (v < 0) { p.print('-'); v = -v; }
  p.print(v / 10000000L); p.print('.');
  int32_t f = v % 10000000L;
  for (int32_t d = 1000000L; d > f && d > 1; d /= 10) p.print('0');
  p.print(f);
}
void printUTC(Print &p) {
  if (!g.timeOK) { p.print(F("--:--:--")); return; }
  print2(p, g.hh); p.print(':'); print2(p, g.mi); p.print(':'); print2(p, g.ss);
}

// ---------------- GPS ----------------
void gpsAirborneMode() {                 // UBX-CFG-NAV5: dynamic model 8 = airborne <4g (fast climbs, 50 km)
  uint8_t msg[44] = { 0xB5, 0x62, 0x06, 0x24, 36, 0, 0x01, 0x00, 8 };   // mask = dynModel only
  uint8_t a = 0, b = 0;
  for (uint8_t i = 2; i < 42; i++) { a += msg[i]; b += a; }
  msg[42] = a; msg[43] = b;
  for (uint8_t i = 0; i < 44; i++) gpsSerial.write(msg[i]);
  delay(50);
}
// ---- compact NMEA parser (GGA + RMC only) - replaces TinyGPS++ to fit the Nano's flash ----
char nb[84]; uint8_t nl = 0;

float fnum(const char *s) {              // small decimal parser (atof costs ~1 KB)
  float v = 0, d = 0; bool neg = (*s == '-'); if (neg) s++;
  for (; *s; s++) { if (*s == '.') d = 1; else if (*s >= '0' && *s <= '9') { if (d) { d *= 10; v += (*s - '0') / d; } else v = v * 10 + (*s - '0'); } }
  return neg ? -v : v;
}
uint32_t unum(const char *s) { uint32_t v = 0; for (; *s >= '0' && *s <= '9'; s++) v = v * 10 + (*s - '0'); return v; }
int32_t coord(const char *s, uint8_t degDigits, char hemi) {   // "ddmm.mmmmm" -> deg x 1e7
  int32_t deg = 0; for (uint8_t i = 0; i < degDigits; i++) deg = deg * 10 + (s[i] - '0');
  s += degDigits;
  int32_t me5 = (s[0] - '0') * 10 + (s[1] - '0'); s += 2;
  uint8_t k = 0; if (*s == '.') s++;
  for (; k < 5; k++) me5 = me5 * 10 + ((*s >= '0' && *s <= '9') ? (*s++ - '0') : 0);
  int32_t v = deg * 10000000L + me5 * 5 / 3;  // minutes x 1e5 -> degrees x 1e7
  return (hemi == 'S' || hemi == 'W') ? -v : v;
}
uint8_t hex(char c) { return c <= '9' ? c - '0' : (c & 0x5F) - 'A' + 10; }

void parseNMEA() {
  char *star = strchr(nb, '*');
  if (!star || nb[0] != '$') return;
  uint8_t cs = 0; for (char *p = nb + 1; p < star; p++) cs ^= *p;
  if (cs != (hex(star[1]) << 4 | hex(star[2]))) return;    // corrupted sentence
  *star = 0;
  char *f[15]; uint8_t n = 0; f[n++] = nb;
  for (char *p = nb; *p && n < 15; p++) if (*p == ',') { *p = 0; f[n++] = p + 1; }
  if (n < 10) return;
  if (strlen(f[1]) >= 6) { g.hh = unum(f[1]) / 10000; g.mi = (unum(f[1]) / 100) % 100; g.ss = unum(f[1]) % 100; g.timeOK = true; }
  if (!strcmp(f[0] + 3, "GGA")) {
    g.fixQ = f[6][0] > '0' && f[2][0];
    if (g.fixQ) {
      g.lat = coord(f[2], 2, f[3][0]); g.lng = coord(f[4], 3, f[5][0]);
      g.sats = unum(f[7]); g.hdop = fnum(f[8]); g.alt = fnum(f[9]);
      g.fixMs = millis(); g.everFix = true;
    }
  } else if (!strcmp(f[0] + 3, "RMC")) {
    g.rmcOK = f[2][0] == 'A';
    if (g.rmcOK) {
      g.mps = fnum(f[7]) * 0.514444; g.courseOK = f[8][0]; g.course = fnum(f[8]);
      g.date = unum(f[9]); g.dateOK = g.date > 0; g.rmcMs = millis();
    }
  }
}
void feedGPS() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (c == '$') nl = 0;
    if (c == '\r' || c == '\n') { if (nl > 6) { nb[nl] = 0; parseNMEA(); } nl = 0; }
    else if (nl < sizeof(nb) - 1) nb[nl++] = c;
  }
}

// ---------------- SD log ----------------
bool sdStart(uint8_t tries) {
  for (uint8_t i = 0; i < tries; i++) { if (SD.begin(SD_CS)) return true; SD.end(); delay(250); }
  return false;
}
void openNewLog(uint8_t tries) {
  sdOK = false;
  if (!sdStart(tries)) { event(F("SD card FAILED")); return; }
  uint16_t n = 1;
  for (; n < 65000; n++) {
    uint16_t v = n;
    logName[0] = 'L'; logName[1] = 'O'; logName[2] = 'G';
    for (int8_t i = 7; i >= 3; i--) { logName[i] = '0' + v % 10; v /= 10; }
    memcpy_P(logName + 8, PSTR(".CSV"), 5);
    if (!SD.exists(logName)) break;
  }
  logFile = SD.open(logName, FILE_WRITE);
  if (!logFile) { event(F("SD file FAILED")); return; }
  sdOK = true;
  logFile.print(F("RFS2027 boat log #")); logFile.print(n);
  logFile.print(F(" - firmware v2.2 - target WP1 ")); printE7(logFile, WAYPOINTS[0].lat_e7);
  logFile.print(' '); printE7(logFile, WAYPOINTS[0].lng_e7); logFile.println();
  logFile.println(F("time_s,utc,mode,motors,event,dist_m,bearing_deg,heading_deg,heading_src,turn_deg,"
                    "speed_kmh,alt_m,sats,hdop,lat,lon,course_deg,compass_deg,mag_x,mag_y,mag_z"));
  logFile.flush();
  Serial.print(F("Logging to ")); Serial.println(logName);
}

// ---------------- mission state ----------------
enum Mode : uint8_t { PAD, FLIGHT, LANDED, SELFCAL, NAVIGATE, ARRIVED };
const char MODE_TXT[][9] PROGMEM = { "ON PAD", "FLIGHT", "LANDED", "SELF-CAL", "NAV", "ARRIVED" };
Mode mode = PAD;
uint8_t wp = 0;
uint32_t modeStart = 0;
void setMode(uint8_t m) { mode = (Mode)m; modeStart = millis(); }   // uint8_t: see setMotors

// launch record in EEPROM so a reset mid-mission resumes instead of waiting on a "pad"
struct FlightRec { uint16_t magic; uint32_t date; uint16_t minute; };
const uint16_t FLIGHT_MAGIC = 0xF117;
bool resumeCheck = false;
uint16_t utcMinute() { return g.hh * 60 + g.mi; }
void saveLaunch() {
  FlightRec r = { FLIGHT_MAGIC, g.dateOK ? g.date : 0, (uint16_t)(g.timeOK ? utcMinute() : 0) };
  EEPROM.put(EE_FLIGHT, r);
}
void clearLaunch() { FlightRec r = { 0, 0, 0 }; EEPROM.put(EE_FLIGHT, r); }

void launched() { event(F("LAUNCH")); saveLaunch(); setMode(FLIGHT); }

bool calibrateOnPad() {                  // blocking 30 s hand calibration
  setMotors(M_STOP);
  Serial.println(F("COMPASS CALIBRATION: hold level + upright, rotate slowly CLOCKWISE 2-3 full circles (30 s)"));
  calStart();
  uint32_t t0 = millis();
  while (millis() - t0 < 30000UL) {
    led(((millis() - t0) % 1000) < 150);
    feedGPS();
    if (magRead()) calSample();
    delay(20);
  }
  led(false); delay(800);
  bool ok = calFinish(1);                 // user rotates clockwise
  if (ok) flashes(3, 700, 400); else flashes(10, 80, 80);
  return ok;
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'L' && mode == PAD) launched();
  }
}

// ---------------- LED status pattern (non-blocking, 2 s cycle) ----------------
bool warn = false, fix = false;
void ledUpdate() {
  uint16_t t = millis() % 2000;
  bool on;
  if (mode == SELFCAL) on = (millis() % 1000) < 150;
  else if (warn && t < 400) on = (t / 50) % 2;
  else if (mode == ARRIVED) on = true;
  else {
    uint8_t n = mode == PAD ? (fix ? 2 : 1) : mode == NAVIGATE ? 3 : 4;
    on = t >= 500 && t < 500 + n * 350 && ((t - 500) % 350) < 175;
  }
  if (on != ledState || (SPCR & _BV(SPE))) led(on);
}

// ---------------- setup ----------------
bool dblResetPending = false;
void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  setMotors(M_STOP);                                         // motors OFF first, always
  pinMode(LED, OUTPUT);
  pinMode(LAUNCH_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  Serial.println(F("\nRFS2027 boat firmware v2.2 starting..."));
  flashes(3, 120, 150);

  bool calRequested = EEPROM.read(EE_DBLRESET) == 0xA5;      // second reset within 5 s
  EEPROM.update(EE_DBLRESET, calRequested ? 0x00 : 0xA5);
  dblResetPending = !calRequested;

  Wire.begin();
  Wire.setWireTimeout(25000, true);
  magInit();
  EEPROM.get(EE_CAL, cal);
  if (cal.magic != CAL_MAGIC) resetCal();

  FlightRec r; EEPROM.get(EE_FLIGHT, r);
  resumeCheck = (r.magic == FLIGHT_MAGIC);

  openNewLog(5);
  gpsSerial.begin(9600);
  gpsAirborneMode();
  if (!magAddr) event(F("Compass MISSING"));
  // pre-launch ritual: calibrate, and ONLY if that succeeds (someone really rotated the boat) clear any
  // old launch record - two power glitches after landing must never make the boat forget it flew
  if (calRequested && calibrateOnPad()) { clearLaunch(); resumeCheck = false; }
  setMode(PAD);
}

// ---------------- loop ----------------
uint32_t lastTick = 0, lastSdTry = 0;
uint8_t tickCount = 0, climbCount = 0, stillCount = 0, pinLowCount = 0, pinHighCount = 0;
bool hadFix = false, havePad = false, pinArmed = false;
float padAlt = 0, peakAlt = -1e6, lastAlt = 0;

void loop() {
  feedGPS();
  handleSerial();
  ledUpdate();
  if (dblResetPending && millis() > 5000) { EEPROM.update(EE_DBLRESET, 0x00); dblResetPending = false; }
  if (millis() - lastTick < 500) return;
  lastTick = millis();
  tickCount++;
  if (!sdOK && mode != NAVIGATE && millis() - lastSdTry > 30000UL) { lastSdTry = millis(); openNewLog(1); }

  // ---- sensors ----
  bool magNow = magRead();
  fix = g.fixQ && millis() - g.fixMs < FIX_TIMEOUT_MS && g.hdop <= MAX_HDOP;
  bool altOK = fix;
  float alt = altOK ? g.alt : NAN;
  bool rmcFresh = g.rmcOK && millis() - g.rmcMs < 2000;
  float spd = rmcFresh ? g.mps : 0;
  bool courseOK = rmcFresh && g.courseOK && spd >= COURSE_MIN_MPS;
  float course = rmcFresh && g.courseOK ? g.course : NAN;
  float hMag = magNow ? magHeading() : NAN;
  float hUsed = courseOK ? course : hMag;
  warn = !magNow || !sdOK || (mode == PAD && !cal.calibrated);
  if (fix && !hadFix) event(F("GPS fix acquired"));
  if (!fix && hadFix) event(F("GPS fix lost"));
  hadFix = fix;

  // ---- resume after an in-mission reset (needs GPS date/time to judge the record's age) ----
  if (resumeCheck && mode == PAD && g.dateOK && g.timeOK && g.date % 100 > 20) {
    FlightRec r; EEPROM.get(EE_FLIGHT, r);
    int16_t age = (int16_t)utcMinute() - (int16_t)r.minute;
    if (r.date == g.date && age >= 0 && age <= (int16_t)RESUME_WINDOW_MIN) {
      event(F("Reset in mission - resuming"));
      setMode(FLIGHT);
      peakAlt = 1e6;                       // descent already happened or will: allow landing
    } else clearLaunch();
    resumeCheck = false;
  }

  // ---- navigation geometry ----
  int32_t lat = 0, lng = 0;
  float dist = NAN, brg = NAN, err = NAN;
  if (g.everFix) { lat = g.lat; lng = g.lng; }
  if (fix && wp < NUM_WAYPOINTS) {
    const float M_PER_E7 = 0.011131949;
    float dN = (WAYPOINTS[wp].lat_e7 - lat) * M_PER_E7;
    float dE = (WAYPOINTS[wp].lng_e7 - lng) * M_PER_E7 * cos(lat * 1e-7 * PI / 180.0);
    dist = sqrt(dN * dN + dE * dE);
    brg = wrap360(atan2(dE, dN) * 180.0 / PI);
    if (!isnan(hUsed)) err = wrap180(brg - hUsed);
  }

  // ---- mission state machine ----
  MotorCmd cmd = M_STOP;
  bool pinLow = digitalRead(LAUNCH_PIN) == LOW;
  switch (mode) {
    case PAD:
      if (pinLow) { if (++pinLowCount >= 4) pinArmed = true; } else pinLowCount = 0;   // pull-pin installed
      if (pinArmed && !pinLow) { if (++pinHighCount >= 2) { launched(); break; } } else pinHighCount = 0;
      if (altOK) {
        if (!havePad) { padAlt = alt; havePad = true; }
        // launch = 2 s of sustained climb on a good fix (a single noisy GPS altitude can't do it)
        bool high = alt - padAlt >= LAUNCH_RISE_M;
        if (high && g.hdop <= 2.5) { if (++climbCount >= 4) { peakAlt = alt; launched(); break; } }
        else if (!high) { climbCount = 0; padAlt += (alt - padAlt) * 0.02; }   // slow follow: car hills can't trigger
      }
      break;

    case FLIGHT:
      if (altOK) {
        peakAlt = max(peakAlt, min(alt, lastAlt));            // peak must be held on 2 readings (ignores spikes)
        bool still = spd < 2.0 && fabs(alt - lastAlt) < 1.0;
        stillCount = still ? stillCount + 1 : 0;
        bool descended = (pinArmed && millis() - modeStart > 60000UL) || (peakAlt - alt >= LANDING_DROP_M);
        if (stillCount >= 20 && descended) { event(F("LANDED")); setMode(LANDED); }
      }
      break;

    case LANDED:
      if (millis() - modeStart >= SETTLE_MS) {
        if (magNow && (long)mz * cal.upZ < 0) { if (tickCount % 20 == 0) event(F("Upside down - waiting")); }
        else {                       // always spin in the water: full calibration if never calibrated,
          calStart(); setMode(SELFCAL); // otherwise just a direction check (pad rotation direction can't break it)
          event(cal.calibrated ? F("Direction check spin") : F("Self-calibration spin"));
        }
      }
      break;

    case SELFCAL:                                            // M_LEFT = right motor only = counter-clockwise spin
      cmd = M_LEFT;
      if (cal.calibrated) {                                  // direction check: track the calibrated angle
        if (magNow) { float a = rawAngle(1); if (zn) sweep += wrap180(a - lastA); lastA = a; zn++; }
        if (fabs(sweep) >= 180 || millis() - modeStart >= 40000UL) {
          if (fabs(sweep) >= 90) {
            int8_t s = sweep < 0 ? 1 : -1;                   // heading must DECREASE while turning left
            if (s != cal.ysign) { cal.ysign = s; cal.aligned = 0; EEPROM.put(EE_CAL, cal); event(F("Compass direction corrected")); }
            else event(F("Compass direction confirmed"));
          }
          event(F("STARTING NAVIGATION")); setMode(NAVIGATE); cmd = M_STOP;
        }
      } else {
        if (magNow) calSample();
        if (millis() - modeStart >= SELFCAL_MS) { calFinish(-1); event(F("STARTING NAVIGATION")); setMode(NAVIGATE); cmd = M_STOP; }
      }
      break;

    case NAVIGATE:
    case ARRIVED:
      if (!fix) break;                                       // no GPS -> stop and wait
      if (courseOK && magNow && motor == M_FWD && !cal.aligned) learnAlignment(course);
      if (mode == ARRIVED) {
        if (dist > RESUME_RADIUS_M) { event(F("Drifted - navigating again")); setMode(NAVIGATE); }
        else break;
      }
      if (dist <= ARRIVE_RADIUS_M) {
        if (wp + 1 < NUM_WAYPOINTS) { wp++; event(F("Waypoint reached")); }
        else { event(F("ARRIVED")); setMode(ARRIVED); break; }
      }
      if (isnan(err)) cmd = M_FWD;
      // keep turning the same way until lined up; "lined up" includes overshooting past zero
      // (one 0.5 s step can turn ~15 deg and skip a narrow window) but not wrapping through 180
      else if (motor == M_LEFT) cmd = (err >= -TURN_OFF_DEG && err < 90) ? M_FWD : M_LEFT;
      else if (motor == M_RIGHT) cmd = (err <= TURN_OFF_DEG && err > -90) ? M_FWD : M_RIGHT;
      else cmd = (fabs(err) > TURN_ON_DEG) ? (err > 0 ? M_RIGHT : M_LEFT) : M_FWD;
      break;
  }
  if (mode == PAD || mode == FLIGHT || mode == LANDED) cmd = M_STOP;   // belt and braces
  setMotors(cmd);
  if (altOK) lastAlt = alt;

  // ---- SD log: one CSV row every 0.5 s ----
  if (sdOK) {
    File &f = logFile;
    f.print(millis() / 1000.0, 1); f.print(','); printUTC(f); f.print(',');
    f.print(FS(MODE_TXT[mode])); f.print(','); f.print(FS(MOTOR_TXT[motor])); f.print(',');
    if (pendingEvent) f.print(pendingEvent);
    f.print(','); if (!isnan(dist)) f.print(dist, 1);
    f.print(','); if (!isnan(brg)) f.print(brg, 0);
    f.print(','); if (!isnan(hUsed)) f.print(hUsed, 0);
    f.print(','); f.print(courseOK ? F("GPS") : magNow ? F("compass") : F("none"));
    f.print(','); if (!isnan(err)) f.print(err, 0);
    f.print(','); f.print(spd * 3.6, 1);
    f.print(','); if (altOK) f.print(alt, 1);
    f.print(','); f.print(g.sats);
    f.print(','); if (g.everFix) f.print(g.hdop, 1);
    f.print(','); if (g.everFix) printE7(f, lat);
    f.print(','); if (g.everFix) printE7(f, lng);
    f.print(','); if (!isnan(course)) f.print(course, 0);
    f.print(','); if (magNow) f.print(hMag, 0);
    f.print(','); f.print(mx); f.print(','); f.print(my); f.print(','); f.println(mz);
    if (tickCount % 4 == 0) {
      f.flush();
      if (f.getWriteError()) { f.close(); SD.end(); sdOK = false; event(F("SD write error")); }
    }
  }
  pendingEvent = nullptr;

  // ---- human-readable serial report once per second ----
  if (tickCount % 2) return;
  Serial.print('['); printUTC(Serial); Serial.print(F("] ")); Serial.print(FS(MODE_TXT[mode]));
  Serial.print(F(" motors ")); Serial.print(FS(MOTOR_TXT[motor]));
  Serial.print(F(" | GPS ")); Serial.print(g.sats); Serial.print(F(" sats"));
  if (altOK) { Serial.print(F(", alt ")); Serial.print(alt, 0); Serial.print(F(" m, ")); Serial.print(spd * 3.6, 1); Serial.print(F(" km/h")); }
  else Serial.print(F(" (no fix)"));
  Serial.print(F("\n   heading ")); if (isnan(hUsed)) Serial.print('?'); else Serial.print(hUsed, 0);
  Serial.print(courseOK ? F(" (GPS)") : F(" (compass)"));
  if (!isnan(dist)) { Serial.print(F(" | target ")); Serial.print(dist, 0); Serial.print(F(" m at ")); Serial.print(brg, 0); }
  if (!isnan(err) && mode >= NAVIGATE) { Serial.print(F(" | turn ")); Serial.print(err, 0); }
  if (warn) Serial.print(!magNow ? F(" | !! COMPASS") : !sdOK ? F(" | !! SD CARD") : F(" | !! NOT CALIBRATED"));
  Serial.println();
}
