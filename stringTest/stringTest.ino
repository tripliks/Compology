#include <SoftwareSerial.h>

SoftwareSerial mySerial(7,8);

void setup()
{

  mySerial.begin(9600);
  byte ID = 1;
  int levelUS = 120;
  int levelIR = 121;
  int temp1 = 20;
  int hum1 = 20;
  int temp2 = 25;
  int battery = 800;

  String postData = "ID" + ID;
  postData += "?levelUS" + levelUS;
	postData += "?levelIR=" + levelIR;
	postData += "?temp1=" + temp1;
	postData += "?hum1=" + hum1;
	postData += "?temp2=" + temp2;
	postData += "?battery=" + battery;
	// softSerial.write(postData);
}

void loop()
{

}