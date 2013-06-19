#include "Timer.h"

#include <DHT22.h>
#include <stdio.h>

#include <SoftwareSerial.h>


// Initialize comm to GSM shield
SoftwareSerial modemSerial(7,8);

// Initialize ultrasonic and variables
SoftwareSerial ultrasonic(10, 12, true); // RX, TX
#define samplePin 11

// Initialize hum and temp sensor and variables
#define DHT22_PIN 5
DHT22 myDHT22(DHT22_PIN);

// Initialize battery
byte batteryPin = A0;


Timer t;
byte sampleEvent;
float sendDataPeriod = 60000;


// Data variables
#define id 1
int us = 120;
int t1 = 20;
int h1 = 20;
int b = 800;

#define bufferSize 100
char string2charBuffer[bufferSize];



void setup()
{
  // Initialize serial debug and modem ports
  Serial.begin(19200);
  modemSerial.begin(9600);
  delay(500);

  Serial.println("Setup BEGIN");

  // Initialize sensors
  initUltrasonic();
  initHumidityAndTemperature();
  initBattery();

  sampleEvent = t.every(sendDataPeriod, sample);

  Serial.println("Setup END");
  
  // First sample for tank calibration
  delay(1000);
  sampleUltrasonic();
}


void loop()
{
  // Run timer update routine, which runs samples after sendDataPeriod
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

  sampleBattery();

  sendData();

  Serial.println("Exit sample loop");
  Serial.println();
}


void sampleUltrasonic()
{
  Serial.println("Ultrasonic BEGIN");
  byte numberSamples = 6;
  String bigBuffer[numberSamples];  // CAN I USE SOMETHING OTHER THAN STRING?

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
  // FIX THIS / MAKE IT SHORTER
  String sendUS = "";
  for (int i=1; i<bigBuffer[numberSamples-1].length(); i++) {
    sendUS += bigBuffer[numberSamples-1][i];
  }
  us = sendUS.toInt();

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
      Serial.print("Temp "); Serial.print(t1); Serial.println(" /10 C");
      Serial.print("Hum "); Serial.print(h1); Serial.println(" /10 %");
      break;
    /*case DHT_ERROR_CHECKSUM:
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
      break;*/
  }

  Serial.println("Hum and temp END");
  Serial.println();
}


void sampleBattery()
{
  Serial.println("Battery BEGIN");

  int batteryReading = analogRead(batteryPin);
  float batteryVoltage = float(batteryReading)*10.0/1023.0 + 0.7;
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
  postData += "&t1=";
  postData += t1;
  postData += "&h1=";
  postData += h1;
  postData += "&b=";
  postData += b;
  postData += "&\"\r";

  postData.toCharArray(string2charBuffer, bufferSize);

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
