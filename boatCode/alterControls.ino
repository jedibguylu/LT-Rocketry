void alterControls(angle, distance){
  if (distance > 0){ 
      if (angle<0) {
          DitgalWrite(D6, HIGH);
          DitgalWrite(D5, LOW);
          DitgalWrite(D8, LOW);
          DitgalWrite(D7,LOW);
      } else if (angle>0) {
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

