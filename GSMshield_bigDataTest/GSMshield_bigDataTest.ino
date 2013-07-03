//Serial Relay - Arduino will patch a 
//serial link between the computer and the modemSerial Shield
//at 19200 bps 8-N-1
//Computer is connected to Hardware UART
//modemSerial Shield is connected to the Software UART 
 
#include <SoftwareSerial.h>
#include <TextFinder.h>
 
SoftwareSerial modemSerial(4, 3);
#define timeout 10
TextFinder finder(modemSerial, timeout);

#define bufferSize 150
char string2charBuffer[bufferSize];


void setup()
{
  modemSerial.begin(9600);               // the modemSerial baud rate   
  Serial.begin(19200);             // the Serial port of Arduino baud rate.
  delay(1000);
}
 
void loop()
{
  Serial.println("START loop");

  modemSerial.write("AT+HTTPTERM\r");
  findOK();

  modemSerial.write("AT+HTTPINIT\r");
  findOK();

  String postData = "AT+HTTPPARA=\"URL\",\"http://compology.herokuapp.com/receive?";
  postData.toCharArray(string2charBuffer, bufferSize);
  modemSerial.write(string2charBuffer);
  // Serial.println(string2charBuffer);

  for ( int i=0; i < 5; i++ )
  {
    postData = "id=";
    for (int j=0; j < 20; j++)
    {
      postData += 111;
      postData += "-";
    }
    postData+=0;
    postData += "&";
    postData.toCharArray(string2charBuffer, bufferSize);
    modemSerial.write(string2charBuffer);
    // Serial.println(string2charBuffer);
    // delay(500);
  }
  
  postData = "\"\r";
  postData.toCharArray(string2charBuffer, bufferSize);
  modemSerial.write(string2charBuffer);

  
  findOK();


  Serial.println("END loop");
}


void findOK()
{
  if (!finder.find("OK")) Serial.println("OK not found!");
}