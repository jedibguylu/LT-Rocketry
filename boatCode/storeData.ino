void storeData(int time, std::vector<double> coords, double distance, int heading, int angle, int turn status, File file, String name){
  String dataString = "";
  dataString += time;
  dataString += " [Coords: ";
  dataString += coords[0];
  dataString += " deg, ";
  dataString += coords[1];
  dataString += " deg | Dist: ";
  dataString += distance;
  dataString += " (unk)| Heading: ";
  dataString += heading;
  dataString += "deg | Angle: ";
  dataString += angle;
  dataString += " deg diff | Turn satus: ";
  dataString += status;

  File file = SD.open(name, FILE_WRITE);

	if (dataFile) {
		dataFile.println(dataString);
		dataFile.close(); // Closing flushes data to physical storage

		Serial.print("Log Successful");
	} else {
		Serial.println("Error opening file for writing!");
	}
}
