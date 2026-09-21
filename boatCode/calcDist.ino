#include <iostream>
#include <cmath>

double calcDist(double boatX, double boatY, double targetX, double targetY){
    double xDist = targetX - boatX;
    double yDist = targetY - boatY;
    double distance = std::sqrt((xDist * xDist) + (yDist * yDist));
    return distance;
}

/**
int main()
{
    std::cout << calcDist(0, 0, 0.5, std::sqrt(3)/2);
    return 0;
}
