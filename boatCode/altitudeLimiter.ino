void altitudeLimiter(){
  delay(600000); //10 min
  int groundlvl = gps.altitude.meters();
  
  while (gps.altitude.meters() < gps.altitude.meters() + 30){
    delay(500);
  }

  while (gps.altitude.meters() > gps.altitude.meters() + 5){
    delay(500);
  }

  delay(60000); //1 min
}
