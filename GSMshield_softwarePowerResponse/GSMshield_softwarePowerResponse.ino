#include <SoftwareSerial.h>
#include <TextFinder.h>
 
SoftwareSerial modemSerial(7, 8);
#define powerPin 9

// Feed TextFinder the stream, modemSerialTimeout (seconds) timeout for searches
#define modemSerialTimeout 10
TextFinder  finder(modemSerial, modemSerialTimeout); 

void setup()
{
  pinMode(powerPin, OUTPUT);
  modemSerial.begin(9600);   
  Serial.begin(19200);
  delay(1000);
  
  Serial.println("Modem ON and response sketch BEGIN");
  Serial.println();
}

void loop()
{
  Serial.println("Begin loop");
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
    if (isConnected()) Serial.println("Connected");
    else Serial.println("Not connected");
  }
  else
  {
    Serial.println("No response from modem");
  }

  // if(isConnected()) Serial.println("Connected!");

  Serial.println("End loop");
  Serial.println();
}


boolean isConnected()
{
  Serial.println("isConnected()");
  modemSerial.write("AT+CREG?\r\n");

  finder.find("+CREG: ");  // Set CREG=1 to return unsolicited network registration code

  // Get network status, could be values 0, 2 (not connected) OR 1 (connected)
  long response = finder.getValue(',');

  // Make sure it didn't time out
  findOK();

  if (response==1) return true;
  else return false;
}


void findOK()
{
  if (!finder.find("OK")) Serial.println("OK not found!");
}