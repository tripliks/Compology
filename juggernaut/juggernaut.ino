#include <DHT22.h>
#include <stdio.h>

#include <SoftwareSerial.h>
#include <TextFinder.h>

#include "LowPower.h"


// Initialize comm to GSM shield
SoftwareSerial modemSerial(7,8);
#define powerPin 9

// Feed TextFinder the stream, modemSerialTimeout (seconds) timeout for searches
#define modemSerialTimeout 10
TextFinder  finder(modemSerial, modemSerialTimeout); 

// Initialize ultrasonic and variables
SoftwareSerial ultrasonic(10, 12, true); // RX, TX
#define samplePin 11

// Initialize hum and temp sensor and variables
#define DHT22_PIN 5
DHT22 myDHT22(DHT22_PIN);

// Initialize battery
byte batteryPin = A0;

#define timeConnectTimeout 5000


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
  modemSerial.begin(9600);
  Serial.begin(19200);
  delay(500);

  Serial.println("Setup BEGIN");

  // Initialize sensors
  initUltrasonic();
  initHumidityAndTemperature();
  initBattery();

  Serial.println("Setup END");
}


void loop()
{
  // Run timer update routine, which runs samples after sendDataPeriod
  sample();

  // Sleep for 60 seconds
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // 8s
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // 16s
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // 24s
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // 32s
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // 40s
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // 48s
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // 56s
  LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF); // 60s
}




//                            //
//  INITIALIZATION FUNCTIONS  //
//                            //
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




//                            //
//       SAMPLE FUNCTIONS     //
//                            //
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

  // Switch to ultrasonic within Software Serial
  ultrasonic.listen();

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

  // Switch to modem within SoftwareSerial
  modemSerial.listen();

  // Check power, power up if necessary
  if ( powerUp() )
  {
    // Check connection, connect if necessary, limit time of attempts
    long timeConnectAttempt = millis();
    while (!isConnected() && ( millis() - timeConnectAttempt ) < timeConnectTimeout) {delay(1000);}

    // Send data if possible
    if (isConnected())
    {
      // Send HTTP data, wait for 200 response
      attemptSendHTTPdata();
    }
    else  // Didn't connect
    {
      Serial.println("Couldn't connect");
    }

    // Power down modem
    powerDown();
  }
  else
  {
    Serial.println("Couldn't powerUp modem");
  }

  Serial.println("Transmission END");
  Serial.println();
}




//                            //
//       MODEM FUNCTIONS      //
//                            //
void attemptSendHTTPdata()
{
  Serial.println("attemptSendHTTPdata()");

  // Open data bearer
  modemSerial.write("AT+SAPBR=1,1\r");
  // Make sure it didn't time out
  findOK();

  // Terminate existing HTTP session??
  modemSerial.write("AT+HTTPTERM\r");
  // Responds with OK if already started, ERROR if none started yet
  delay(1000);

  // Start HTTP session
  modemSerial.write("AT+HTTPINIT\r");
  findOK();

  // Create HTTP GET string - OPTIMIZE, maybe with replace string?
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
  findOK();

  // Set HTTP bearer
  modemSerial.write("AT+HTTPPARA=\"CID\",1\r");
  findOK();

  // Initiate GET request
  modemSerial.write("AT+HTTPACTION=0\r");
  findOK();

  // Look for HTTP GET 200 reponse
  // +HTTPACTION:#,#(HTTP CODE),#(BYTES?)
  finder.find("+HTTPACTION:0,");
  long HTTPcode = finder.getValue();
  Serial.print("HTTPcode "); Serial.println(HTTPcode);
  // return true;
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


boolean powerUp()
{
  Serial.println("powerUp()");
  toggleModemPower();
  finder.find(".........");
  // Call Ready can take 10 seconds
  if (finder.find("Call Ready"))
  {
    // Successfully turned on
    Serial.println("Modem on and responsive");
    return true;
  }
  else
  {
    Serial.println("No response from modem");
    return false;
  }
}

void powerDown()
{
  Serial.println("powerDown()");
  toggleModemPower();
}

void toggleModemPower()
{
  digitalWrite(9,HIGH);
  delay(1500);
  digitalWrite(9,LOW);
}


void findOK()
{
  if (!finder.find("OK")) Serial.println("OK not found!");
}

