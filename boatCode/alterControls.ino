#include <cmath>

int alterControls(int angle, double distance){
  if (std::abs(distance) > 0.00001){ //should be ~1m
      if (angle < 0 && std::abs(angle) > 5) {
          DitgalWrite(D6, HIGH);
          DitgalWrite(D5, LOW);
          DitgalWrite(D8, LOW);
          DitgalWrite(D7,LOW);
          return -1;
      } else if (angle > 0 && std::abs(angle) > 5) {
          DitgalWrite(D8, HIGH);
          DitgalWrite(D7,LOW);
          DitgalWrite(D6, LOW);
          DitgalWrite(D5, LOW);
          return 1;
      } else {
          DitgalWrite(D8, HIGH);
          DitgalWrite(D7, LOW);
          DitgalWrite(D6, HIGH);
          DitgalWrite(D5, LOW);
          return 0;
      }
  } else {
      DitgalWrite(D8, LOW);
      DitgalWrite(D7, LOW);
      DitgalWrite(D6, LOW);
      DitgalWrite(D5, LOW);
      return 0;
  }
}

