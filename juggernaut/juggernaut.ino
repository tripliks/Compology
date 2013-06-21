#include <DHT22.h>
#include <stdio.h>

#include <SoftwareSerial.h>
#include <TextFinder.h>

#include <LowPower.h>


// Initialize comm to GSM shield
SoftwareSerial modemSerial(4,3);
#define powerPin 2

// Feed TextFinder the stream, modemSerialTimeout (seconds) timeout for searches
#define modemSerialTimeout 10
TextFinder  finder(modemSerial, modemSerialTimeout); 

// Initialize ultrasonic and variables
SoftwareSerial ultrasonic(11, 13, true); // RX, TX
#define samplePin 10

// Initialize hum and temp sensor and variables
#define DHT22_PIN 12
DHT22 myDHT22(DHT22_PIN);

// Initialize battery
byte batteryPin = A0;

#define timeConnectTimeout 10000
#define timeAttachTimeout 10000


// Data variables
#define id 3
int us = 120;
int ir = 0;
int t1 = 20;
int h1 = 20;
int t2 = 0;
int h2 = 0;
int b = 800;

#define bufferSize 150
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

  delay(3000);
  Serial.println("Setup END");
}


void loop()
{
  // Run timer update routine, which runs samples after sendDataPeriod
  sample();
  sendData();
  delay(2000);

  // Sleep for 60 seconds

  Serial.println("Going to sleep");
  delay(1000);
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // 8s
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // 16s
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // 24s
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // 32s
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // 40s
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // 48s
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // 56s
  LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF); // 60s
  Serial.println("Waking up");
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
      // Check connection, connect if necessary, limit time of attempts
      long timeAttachAttempt = millis();
      while (!isAttached() && ( millis() - timeAttachAttempt ) < timeAttachTimeout) {delay(1000);}

      if (isAttached())
      {
        // Send HTTP data, wait for 200 response
        attemptSendHTTPdata();
      }
      else // Didn't attach
      {
        Serial.println("Failed to attach");
      }


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

  // Check bearer
  modemSerial.write("AT+SAPBR=2,1\r");
  finder.find("+SAPBR: 1,");
  long bearerStatus = finder.getValue();
  Serial.println(bearerStatus);
  findOK();
  if ( bearerStatus == 3 )
  {
    // Open data bearer
    modemSerial.write("AT+SAPBR=1,1\r");
    // Make sure it didn't time out
    findOK();
  }
  else if ( bearerStatus == 1 ) {}
  else
  {
    Serial.println("Weird bearer status, breaking send attempt");
    return;
  }

  // Start HTTP session
  modemSerial.write("AT+HTTPINIT\r");
  findOK();

  // Create HTTP GET string - OPTIMIZE, maybe with replace string?
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
  postData += "&h2=";
  postData += h2;
  postData += "&t2=";
  postData += t2;
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


boolean isAttached()
{
  // Check on network attachment
  modemSerial.write("AT+CGATT?\r\n");
  finder.find("+CGATT: ");
  long attachStatus = finder.getValue();
  findOK();
  if ( attachStatus == 1 ) return 1;
  else return 0;
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
  delay(1000);
}

void toggleModemPower()
{
  digitalWrite(powerPin,HIGH);
  delay(1500);
  digitalWrite(powerPin,LOW);
}


void findOK()
{
  if (!finder.find("OK")) Serial.println("OK not found!");
}

