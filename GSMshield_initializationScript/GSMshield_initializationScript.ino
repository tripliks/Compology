#include <SoftwareSerial.h>
#include <TextFinder.h>
 
SoftwareSerial modemSerial(4, 3);
#define powerPin 2

// Feed TextFinder the stream, modemSerialTimeout (seconds) timeout for searches
#define modemSerialTimeout 15
TextFinder  finder(modemSerial, modemSerialTimeout); 

void setup()
{
  pinMode(powerPin, OUTPUT);
  modemSerial.begin(9600);   
  Serial.begin(19200);
  delay(1000);
  
  Serial.println("Modem ON and response sketch BEGIN");
  Serial.println();

  powerUp();
  delay(5000);

  // Set baud rate
  modemSerial.write("AT+IPR=9600\r");
  findOK();
  delay(1000);

  // Turn off echo
  modemSerial.write("ATEO\r");
  findOK();

  // Turn on error codes
  modemSerial.write("AT+CMEE=1\r");
  findOK();

  // Add tMobile PDP context
  modemSerial.write("AT+CGDCONT=1,\"IP\",\"epc.tmobile.com\"\r");
  findOK();

  // Turn on error codes
  modemSerial.write("AT+CGATT=1\r");
  findOK();
  delay(1000);

  Serial.println("Enter command mode");

}

void loop()
{
  String fromGSM = "";
  if (modemSerial.available())              // if date is comming from softwareserial port ==> data is comming from modemSerial shield
  {
    while(modemSerial.available()>0)          // reading data into char array 
    {
      char c = modemSerial.read();
      fromGSM += c;
    }
    Serial.print(fromGSM);            // if no data transmission ends, write buffer to hardware serial port
  }
  
  if (Serial.available())            // if data is available on hardwareserial port ==> data is comming from PC or notebook
  {
    modemSerial.write(Serial.read());       // write it to the modemSerial shield
  }
}




void findOK()
{
  if (!finder.find("OK")) Serial.println("OK not found!");
}

void powerUp()
{
  // Toggle modem power
  digitalWrite(powerPin,HIGH);
  delay(1500);
  digitalWrite(powerPin,LOW);
  Serial.println("Toggle complete");
  finder.find(".........");

  // Call Ready can take 10 seconds
  if (finder.find("Call Ready"))
  {
    // Successfully turned on
    Serial.println("Modem on and responsive");
  }
  else
  {
    Serial.println("No response from modem");
  }
}