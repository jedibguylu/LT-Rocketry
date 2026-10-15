double calcDist(double boatLat, double boatLon, double targetLat, double targetLon){
    double lat1 = boatLat * radConvert;
    double lat2 = targetLat * radConvert;
    double dLat = (targetLat - boatLat) * radConvert;
    double dLon = (targetLon - boatLon) * radConvert;

    // Haversine calculation
    double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
               cos(lat1) * cos(lat2) *
               sin(dLon / 2.0) * sin(dLon / 2.0);
               
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return R * c; // Distance in meters
}
