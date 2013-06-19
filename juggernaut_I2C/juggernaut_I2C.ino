#include "Timer.h"

//#include <DistanceGP2Y0A21YK.h>

#include <I2C.h>
#include <MMA8453_n0m1.h>

#include <DHT22.h>
#include <stdio.h>

#include <SoftwareSerial.h>


// Initialize comm to GSM shield
SoftwareSerial modemSerial(7,8);

// Initialize accelerometer and variables
MMA8453_n0m1 accel;

// Initialize ultrasonic and variables
#include "Wire.h"
#define SensorAddress byte(0x70)
#define RangeCommand byte(0x51)
#define ChangeAddressCommand1 byte(0xAA)
#define ChangeAddressCommand2 byte(0xA5)

// Initialize IR sensor
//DistanceGP2Y0A21YK infrared;

// Initialize hum and temp sensor and variables
#define DHT22_PIN 5
DHT22 myDHT22(DHT22_PIN);

// Initialize battery
byte batteryPin = A0;


Timer t;
byte sampleEvent;
float sendDataPeriod = 60000;


// Data variables
const byte id = 2;
int us = 120;
int ir = 121;
int t1 = 20;
int h1 = 20;
int t2 = 25;
int b = 800;

char string2charBuffer[200];



void setup()
{
  // Initialize serial debug output
  Serial.begin(19200);

  modemSerial.begin(9600);
  delay(500);

  Serial.println("Setup BEGIN");

  initUltrasonic();
//  initInfrared();
  initHumidityAndTemperature();
//  initAcceleration();
  initBattery();

  sampleEvent = t.every(sendDataPeriod, sample);

  Serial.println("Setup END");
  
  // First sample for tank calibration
  delay(1000);
  sampleUltrasonic();
}


void loop()
{
  t.update();
}




void initUltrasonic()
{
  Wire.begin();
  delay(100);
}

void initInfrared()
{
//  infrared.begin(A2);
}

void initHumidityAndTemperature()
{

}

void initAcceleration()
{
  // accel.setI2CAddr(0x1D); //change your device address if necessary, default is 0x1C
  // accel.dataMode(true, 2); //enable highRes 10bit, 2g range [2g,4g,8g]
  // accel.motionMode(8,true,true,true,false,2);  // Arduino interrupt pin 2
}

void initBattery()
{
  pinMode(batteryPin, INPUT);
}




void sample()
{
  Serial.println();
  Serial.println("----------");
  Serial.println("Enter sample loop");
  Serial.println();

  sampleUltrasonic();
  
//  sampleInfrared();

  sampleHumidityAndTemperature();

//  sampleAcceleration();

  sampleBattery();

  sendData();

  Serial.println("Exit sample loop");
  Serial.println();
}


void sampleUltrasonic()
{
  Serial.println("Ultrasonic BEGIN");
  byte numberSamples = 6;
  word bigBuffer[numberSamples];

  // Take numberSamples and put into bigBuffer
  for (byte sampleNum = 0; sampleNum < numberSamples; sampleNum++)
  {
    takeRangeReading();                                       //Tell the sensor to perform a ranging cycle
    delay(100);                                                    //Wait for the sensor to finish
    bigBuffer[sampleNum] = requestRange();                           //Get the range from the sensor
    Serial.println("Sample");
  }

  // Print results to serial
  for (byte i=0; i<numberSamples; i++)
  {
    Serial.println(bigBuffer[i]);
  }

  // Store value for sending to server
  us = bigBuffer[numberSamples-1];
//  Serial.println("sendUS below");
//  Serial.println(sendUS);
//  us = sendUS;
  
  Serial.println("Ultrasonic END");
  Serial.println();
}


void sampleInfrared()
{
  Serial.println("Infrared BEGIN");
  
//  ir = infrared.getDistanceRaw();
//  Serial.print("Infrared analog reading "); Serial.println(ir);
  
  Serial.println("Infrared END");
  Serial.println();
}


void sampleHumidityAndTemperature()
{
  Serial.println("Hum and temp BEGIN");

  DHT22_ERROR_t errorCode;
  errorCode = myDHT22.readData();
  switch(errorCode)
  {
    case DHT_ERROR_NONE:
      t1 = myDHT22.getTemperatureCInt();
      h1 = myDHT22.getHumidityInt();
      Serial.print("Temp "); Serial.print(t1); Serial.println(" /10 C");
      Serial.print("Hum "); Serial.print(h1); Serial.println(" /10 %");
      break;
    case DHT_ERROR_CHECKSUM:
      Serial.print("check sum error ");
      break;
    case DHT_BUS_HUNG:
      Serial.println("BUS Hung ");
      break;
    case DHT_ERROR_NOT_PRESENT:
      Serial.println("Not Present ");
      break;
    case DHT_ERROR_ACK_TOO_LONG:
      Serial.println("ACK time out ");
      break;
    case DHT_ERROR_SYNC_TIMEOUT:
      Serial.println("Sync Timeout ");
      break;
    case DHT_ERROR_DATA_TIMEOUT:
      Serial.println("Data Timeout ");
      break;
    case DHT_ERROR_TOOQUICK:
      Serial.println("Polled to quick ");
      break;
  }

  Serial.println("Hum and temp END");
  Serial.println();
}


void sampleAcceleration()
{
  Serial.println("Acceleration BEGIN");
  // accel.update();
  // if (accel.motion()) Serial.println("Motion!");
  Serial.println("Acceleration END");
  Serial.println();
}

void sampleBattery()
{
  Serial.println("Battery BEGIN");

  int batteryReading = analogRead(batteryPin);
  float batteryVoltage = float(batteryReading)*10.0/1023.0;
  Serial.print(batteryVoltage); Serial.println(" Volts");

  // Store value for sending to server
  b = batteryReading;

  Serial.println("Battery END");
  Serial.println();
}


void sendData()
{
  Serial.println("Transmission BEGIN");


  modemSerial.write("AT+SAPBR=1,1\r");
  delay(5000);

  modemSerial.write("AT+HTTPTERM\r");
  delay(1000);
  modemSerial.write("AT+HTTPINIT\r");
  delay(1000);

  String postData = "";
  postData += id;
  postData += "&us=";
  postData += us;
  postData += "&ir=";
  postData += ir;
  postData += "&t1=";
  postData += t1;
  postData += "&h1=";
  postData += h1;
  postData += "&t2=";
  postData += t2;
  postData += "&b=";
  postData += b;
  postData += "&\"\r";

  Serial.print("AT+HTTPPARA=\"URL\",\"http://compology.herokuapp.com/receive?id=");
  Serial.println(postData);
  postData.toCharArray(string2charBuffer, 200);

  modemSerial.write("AT+HTTPPARA=\"URL\",\"http://compology.herokuapp.com/receive?id=");
  modemSerial.write(string2charBuffer);
  Serial.println(string2charBuffer);
  delay(1000);

  modemSerial.write("AT+HTTPPARA=\"CID\",1\r");
  delay(1000);

  modemSerial.write("AT+HTTPACTION=0\r");
  delay(1000);

  Serial.println("Transmission END");
  Serial.println();
}




//Commands the sensor to take a range reading
void takeRangeReading(){
       Wire.beginTransmission(SensorAddress);             //Start addressing 
       Wire.write(RangeCommand);                             //send range command 
       Wire.endTransmission();                                  //Stop and do something else now
}    

//Returns the last range that the sensor determined in its last ranging cycle in centimeters. Returns 0 if there is no communication. 
word requestRange(){ 
    Wire.requestFrom(SensorAddress, byte(2));
            if(Wire.available() >= 2){                            //Sensor responded with the two bytes 
           byte HighByte = Wire.read();                        //Read the high byte back 
           byte LowByte = Wire.read();                        //Read the low byte back 
           word range = word(HighByte, LowByte);         //Make a 16-bit word out of the two bytes for the range 
           return range; 
        }
        else { 
        return word(0);                                             //Else nothing was received, return 0 
    }
}
