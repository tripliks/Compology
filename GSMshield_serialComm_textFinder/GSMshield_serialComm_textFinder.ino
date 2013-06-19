#include <SoftwareSerial.h>
#include <TextFinder.h>

SoftwareSerial GPRS(7, 8);

TextFinder  finder(GPRS, 2);  

 

void setup()
{
  GPRS.begin(9600);               // the GPRS baud rate   
  Serial.begin(19200);             // the Serial port of Arduino baud rate.
//  Serial.println("GSM comm BEGIN");
  
  // Sleep code for benchmarking current draw
//  delay(5000);
//  GPRS.write("AT+CSCLK=1");
  GPRS.write("AT+CREG=0\r\n");
  delay(1000);
}
 
void loop()
{
  
  if ( isConnected() ) Serial.println("Connected!");
  else Serial.println("Not connected.");
  delay(5000);
}


boolean isConnected()
{
  GPRS.write("AT+CREG?\r\n");
  Serial.write("AT+CREG?\r\n");
  finder.find("+CREG: ");  // Set CREG=1 to return unsolicited network registration code
  long response = finder.getValue(',');
  if (finder.find("OK"))
  {
    Serial.println("OK found!");
  }
  else
  {
    Serial.println("OK not found!");
  }
  if (response==1) return true;
  else return false;
}
