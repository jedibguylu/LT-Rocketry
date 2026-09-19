#include <iostream>
#include <string>
#include <cmath>
#include <numbers>


int calcAngle(double boatX, double boatY, double targetX, double targetY, int heading) {
    double radConvert = 3.14159 / 180.0; //from deg to rad
    double degConvert = 180.0 / 3.14159; //from rad to deg
    
    // https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
    double dx = std::cos(targetX * radConvert) * std::sin((targetY - boatY) * radConvert);
    double dy = std::cos(boatX * radConvert) * std::sin(targetX * radConvert) - std::sin(boatX * radConvert) * std::cos(targetX * radConvert) * std::cos((targetY - boatY) * radConvert);
    
    std::cout << dx << "\n";
    std::cout << dy << "\n";

    // Calculate angle using atan2(y, x)
    double bearing = std::atan2(dx, dy) * degConvert;
    
    std::cout << bearing << "\n";

    // Normalize bearing to [0, 360)
    int course = static_cast<int>(std::fmod(bearing + 360.0, 360.0));

    std::cout << course << "\n";

    // Calculate turn difference relative to current heading (-180 to +180)
    int difference = course - heading;
    while (difference > 180)  difference -= 360;
    while (difference < -180) difference += 360;

    return difference;
}

/**
int main() {
    std::cout << "boatX ";
    double boatX;
    std::cin >> boatX;    
    std::cout << "boatY ";
    double boatY;
    std::cin >> boatY;  
    std::cout << "targetX ";
    double targetX;
    std::cin >> targetX;  
    std::cout << "targetY ";
    double targetY;
    std::cin >> targetY;  
    std::cout << "current heading ";
    double heading;
    std::cin >> heading;
    
    //test coordinates
    //double boatX = 41.967024467315724;
    //double boatY = -87.69687476523706;
    //double targetX = 29.76441958947544;
    //double targetY = -95.36534692173001;
    //int heading = 270;
    
    int angle = calcAngle(boatX, boatY, targetX, targetY, heading);
    
    std::cout << angle;
    
    return 0;
}
