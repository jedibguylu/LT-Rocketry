std::vector<int> getCoords() {
  std::vector<int> point; 
  double lat, lon;
  
  if  (gps.location.isUpdated()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
  }

  point.push_back(lat);
  point.push_back(lon);
  
  return point;
}
