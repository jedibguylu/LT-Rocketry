#include <cmath>

void alterControls(int angle, int distance){
  if (std::abs(distance) > 0.00001){ //should be ~1m
      if (angle<0 && std::abs(angle) > 5) {
          DitgalWrite(D6, HIGH);
          DitgalWrite(D5, LOW);
          DitgalWrite(D8, LOW);
          DitgalWrite(D7,LOW);
      } else if (angle>0 && std::abs(angle) > 5) {
          DitgalWrite(D8, HIGH);
          DitgalWrite(D7,LOW);
          DitgalWrite(D6, LOW);
          DitgalWrite(D5, LOW);
      } else {
          DitgalWrite(D8, HIGH);
          DitgalWrite(D7, LOW);
          DitgalWrite(D6, HIGH);
          DitgalWrite(D5, LOW);
      }
  } else {
      DitgalWrite(D8, LOW);
      DitgalWrite(D7, LOW);
      DitgalWrite(D6, LOW);
      DitgalWrite(D5, LOW);
  }
  
  return 0;
}

