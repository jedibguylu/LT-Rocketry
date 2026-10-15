int alterControls(int angle, double distance){
  if (abs(distance) > 0.00001){ //should be ~1m
      if (angle < 0 && abs(angle) > 5) {
          digitalWrite(6, HIGH);
          digitalWrite(5, LOW);
          digitalWrite(8, LOW);
          digitalWrite(7,LOW);
          return -1;
      } else if (angle > 0 && abs(angle) > 5) {
          digitalWrite(8, HIGH);
          digitalWrite(7,LOW);
          digitalWrite(6, LOW);
          digitalWrite(5, LOW);
          return 1;
      } else {
          digitalWrite(8, HIGH);
          digitalWrite(7, LOW);
          digitalWrite(6, HIGH);
          digitalWrite(5, LOW);
          return 0;
      }
  } else {
      digitalWrite(8, LOW);
      digitalWrite(7, LOW);
      digitalWrite(6, LOW);
      digitalWrite(5, LOW);
      return 0;
  }
}
