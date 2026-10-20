int calcAngle(double boatLat, double boatLon, double targetLat, double targetLon, int heading) {
    // https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
    double dx = cos(targetLat * radConvert) * sin((targetLon - boatLon) * radConvert);
    double dy = cos(boatLat * radConvert) * sin(targetLat * radConvert) - sin(boatLat * radConvert) * cos(targetLat * radConvert) * cos((targetLon - boatLon) * radConvert);
    
    //cout << dx << "\n";
    //cout << dy << "\n";

    // Calculate angle using atan2(y, x)
    double bearing = (atan2(dx, dy) * degConvert) + 6;
    
    //cout << bearing << "\n";

    // Normalize bearing to [0, 360)
    int course = static_cast<int>(fmod(bearing + 360.0, 360.0));

    //cout << course << "\n";

    // Calculate turn difference relative to current heading (-180 to +180)
    int difference = course - heading;
    while (difference > 180)  difference -= 360;
    while (difference < -180) difference += 360;

    return difference;
}
