void storeData(int time, Point coords, double distance, int heading, int angle, int turnStatus, String name){
  String dataString = "";
  dataString += time;
  dataString += " [Coords: ";
  dataString += coords.lat;
  dataString += " deg, ";
  dataString += coords.lng;
  dataString += " deg | Dist: ";
  dataString += distance;
  dataString += " (unk)| Heading: ";
  dataString += heading;
  dataString += "deg | Angle: ";
  dataString += angle;
  dataString += " deg diff | Turn satus: ";
  dataString += turnStatus;

  if (!testMode){
    File dataFile = SD.open(name, FILE_WRITE);

    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close(); // Closing flushes data to physical storage
      Serial.println(dataString);
    } else {
      Serial.println("Error opening file for writing!");
    }
  } else {
    Serial.println(dataString);
  }
  
	return 0;
}
