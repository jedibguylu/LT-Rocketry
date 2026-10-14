std::vector<double> getCoords() {
  double lat, lon;
  
  if  (gps.location.isUpdated()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
  }

  std::vector<double> point = {lat, lon};
  
  return point;
}
