Point getCoords() {
  Point p;
  
  if (!testMode){
    if (gps.location.isUpdated()) {
    p.lat = gps.location.lat();
    p.lng = gps.location.lng();
    }
  } else {
    p.lat = testBoat.lat;
    p.lng = testBoat.lng;
  }
  
  return p;
}
