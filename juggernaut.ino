#include "Timer.h"

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
SoftwareSerial ultrasonic(10, 12, true); // RX, TX
byte samplePin = 11;

// Initialize hum and temp sensor and variables
#define DHT22_PIN 5
DHT22 myDHT22(DHT22_PIN);

// Initialize battery
byte batteryPin = A0;


Timer t;
byte sampleEvent;


// Data variables
const byte id = 1;
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
  initHumidityAndTemperature();
  initAcceleration();
  initBattery();

  sampleEvent = t.every(15000, sample);

  Serial.println("Setup END");
}


void loop()
{
  t.update();
}




void initUltrasonic()
{
  pinMode(samplePin, OUTPUT);
  digitalWrite(samplePin, LOW);
  ultrasonic.begin(9600);
  delay(100);
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

  sampleHumidityAndTemperature();

  sampleAcceleration();

  sampleBattery();

  sendData();

  Serial.println("Exit sample loop");
  Serial.println();
}


void sampleUltrasonic()
{
  Serial.println("Ultrasonic BEGIN");
  byte numberSamples = 6;
  String bigBuffer[numberSamples];

  // Take numberSamples and put into bigBuffer
  for (byte sampleNum = 0; sampleNum < numberSamples; sampleNum++)
  {
    // Tell sensor to sample
    // Drive sample pin to HIGH for for at least 20 us, return to LOW
    digitalWrite(samplePin, HIGH);
    delay(150);
    digitalWrite(samplePin, LOW);

    // Read result from serial
    String smallBuffer;
    while (ultrasonic.available())
    {
      char c = ultrasonic.read();
      smallBuffer+=c;
    }
    bigBuffer[sampleNum] = smallBuffer;
    Serial.println("Sample");
  }

  // Store value for sending to server
  us = bigBuffer[numberSamples-1].toInt();

  // Print results to serial
  for (byte i=0; i<numberSamples; i++)
  {
    Serial.println(bigBuffer[i]);
  }
  Serial.println("Ultrasonic END");
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

  String postData = "AT+HTTPPARA=\"URL\",\"http://compology.herokuapp.com/receive?id=";
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

  postData.toCharArray(string2charBuffer, 200);

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