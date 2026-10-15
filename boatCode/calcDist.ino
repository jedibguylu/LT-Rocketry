double calcDist(double boatX, double boatY, double targetX, double targetY){
    double xDist = targetX - boatX;
    double yDist = targetY - boatY;
    double distance = sqrt((xDist * xDist) + (yDist * yDist));
    return distance;
}
