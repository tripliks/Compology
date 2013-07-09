//Serial Relay - Arduino will patch a 
//serial link between the computer and the GPRS Shield
//at 19200 bps 8-N-1
//Computer is connected to Hardware UART
//GPRS Shield is connected to the Software UART 
 
#include <SoftwareSerial.h>
 
//SoftwareSerial GPRS(4, 3);
SoftwareSerial GPRS(7, 8);


void setup()
{
  GPRS.begin(9600);               // the GPRS baud rate   
  Serial.begin(19200);             // the Serial port of Arduino baud rate.
//  Serial.println("GSM comm BEGIN");
  
  // Sleep code for benchmarking current draw
//  delay(5000);
//  GPRS.write("AT+CSCLK=1");
}
 
void loop()
{
  String fromGSM = "";
  if (GPRS.available())              // if date is comming from softwareserial port ==> data is comming from gprs shield
  {
    while(GPRS.available()>0)          // reading data into char array 
    {
      char c = GPRS.read();
      fromGSM += c;
    }
    Serial.print(fromGSM);            // if no data transmission ends, write buffer to hardware serial port
  }
  
  if (Serial.available())            // if data is available on hardwareserial port ==> data is comming from PC or notebook
  {
    GPRS.write(Serial.read());       // write it to the GPRS shield
  }
}
