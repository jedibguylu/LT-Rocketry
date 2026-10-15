int calcAngle(double boatX, double boatY, double targetX, double targetY, int heading) {
    double radConvert = 3.14159 / 180.0; //from deg to rad
    double degConvert = 180.0 / 3.14159; //from rad to deg
    
    // https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
    double dx = cos(targetX * radConvert) * sin((targetY - boatY) * radConvert);
    double dy = cos(boatX * radConvert) * sin(targetX * radConvert) - sin(boatX * radConvert) * cos(targetX * radConvert) * cos((targetY - boatY) * radConvert);
    
    //cout << dx << "\n";
    //cout << dy << "\n";

    // Calculate angle using atan2(y, x)
    double bearing = atan2(dx, dy) * degConvert;
    
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
