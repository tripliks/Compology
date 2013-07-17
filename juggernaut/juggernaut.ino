#include <DHT22.h>
#include <stdio.h>
#include <SoftwareSerial.h>
#include <TextFinder.h>
#include <LowPower.h>



// UNIT NAME
#define id 4

// Which ultrasonic sensor?  TTL or I2C?  UNCOMMENT NEXT 2 LINES FOR AN I2C SENSOR
#define I2Cultrasonic_flag
#include <Wire.h>



// Initialize comm to GSM shield
SoftwareSerial modemSerial(4,3);
#define powerPin 2


// Feed TextFinder the stream, modemSerialTimeout (seconds) timeout for searches
#define modemSerialTimeout 35
TextFinder  finder(modemSerial, modemSerialTimeout); 


// Initialize ultrasonic and variables
#ifndef I2Cultrasonic_flag
  SoftwareSerial ultrasonic(11, 13, true); // RX, TX
  #define samplePin 10
#endif

#define SensorAddress byte(0x70)
#define RangeCommand byte(0x51)
#define ChangeAddressCommand1 byte(0xAA)
#define ChangeAddressCommand2 byte(0xA5)

#define USnumberSamples 9
bool levelChange_flag = false;
#define USlevelChangeThreshold 3
int lastUSsample = 0;


// Initialize hum and temp sensor and variables
#define DHT22_PIN 12
DHT22 myDHT22(DHT22_PIN);


// Initialize battery
byte batteryPin = A3;


// Connection timeouts for the first connection
#define timeConnectTimeout 15000
#define timeAttachTimeout 15000


// Data variables
#define numSampleBuffer 12 // Same as number of samples per send
int us[numSampleBuffer];
int ir[numSampleBuffer];
int t1[numSampleBuffer];
int h1[numSampleBuffer];
int t2[numSampleBuffer];
int h2[numSampleBuffer];
int b[numSampleBuffer];
int csq;

#define bufferSize 75
char string2charBuffer[bufferSize];

int numSamplesSinceLastSendData = numSampleBuffer-1; // Initialize to this value so upon first loop, it sends data
long sendDataTimer = 0;




void setup()
{
  // Initialize serial debug and modem ports
  modemSerial.begin(9600);
  Serial.begin(19200);
  delay(500);

  // Serial.println("Setup BEGIN");

  Serial.println();
  Serial.print("Unit Name "); Serial.println(id);
  Serial.println();

  // Initialize data storage
  emptyDataStorage();

  // Initialize sensors
  initUltrasonic();
  initHumidityAndTemperature();
  initBattery();

  delay(2000);
  // Serial.println("Setup END");
  Serial.println();
}


void loop()
{
  // Run timer update routine, which runs samples after sendDataPeriod
  sample();
  numSamplesSinceLastSendData++;
  Serial.print("Samples "); Serial.println(numSamplesSinceLastSendData);
  if ( numSamplesSinceLastSendData == numSampleBuffer || levelChange_flag == true )
  {
    numSamplesSinceLastSendData = 0;
    levelChange_flag = false;
    sendData();
  }
  delay(1000);

  // Go to sleep for 5 minutes = 300 seconds
  Serial.println("Sleep");
  delay(500);  // Allow Serial to post before sleeping
  for ( int counter=0; counter < 37; counter++ ) { LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); }// sleeps for 296 s
  LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);  // sleeps to 300 s
  // Wake up
  Serial.println("Wake");
}




//                            //
//  INITIALIZATION FUNCTIONS  //
//                            //
void emptyDataStorage()
{
  setArray(us, 0);
  setArray(ir, 0);
  setArray(t1, 0);
  setArray(h1, 0);
  setArray(t2, 0);
  setArray(h2, 0);
  setArray(b, 0);
}


void initUltrasonic()
{
  #ifndef I2Cultrasonic_flag
    pinMode(samplePin, OUTPUT);
    digitalWrite(samplePin, LOW);
    ultrasonic.begin(9600);
  #endif

  #ifdef I2Cultrasonic_flag
    Wire.begin();
  #endif

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
  Serial.println("-----");
  Serial.println("Sample BEGIN");
  Serial.println();

  sampleUltrasonic();

  sampleHumidityAndTemperature();

  sampleBattery();

  Serial.println("Sample END");
  Serial.println();
}


void sampleUltrasonic()
{
  // Serial.println("Ultrasonic BEGIN");

  #ifndef I2Cultrasonic_flag
    // Switch to ultrasonic within Software Serial
    ultrasonic.listen();
  #endif

  int bigBuffer[USnumberSamples];
  int runningTotal = 0;

  // Take numberSamples and put into bigBuffer
  for (byte sampleNum = 0; sampleNum < USnumberSamples; sampleNum++)
  {

    // Tell sensor to sample and record reading in bigBuffer

    #ifndef I2Cultrasonic_flag
      // Drive sample pin to HIGH for for at least 20 us, return to LOW
      digitalWrite(samplePin, HIGH);
      delay(150);
      digitalWrite(samplePin, LOW);

      // Read result from serial
      String smallBuffer;
      // Serial.print("char storage ");
      while (ultrasonic.available())
      {
        char c = ultrasonic.read();
        // Serial.print(c);
        smallBuffer += c;
      }
      // Serial.println();
      smallBuffer.replace("R", NULL);
      bigBuffer[sampleNum] = smallBuffer.toInt();
    #endif

    #ifdef I2Cultrasonic_flag
      takeRangeReading();                                       //Tell the sensor to perform a ranging cycle
      delay(100);                                                    //Wait for the sensor to finish
      bigBuffer[sampleNum] = requestRange();
    #endif


    // Ignore 3 samples, but keep running total of next 6 for the average
    if ( sampleNum >= 3) runningTotal += bigBuffer[sampleNum];


    // Print individual samples
    Serial.print("US "); Serial.println(bigBuffer[sampleNum]);

  }

  // Calculate average
  int USsample = float(runningTotal)/6.0;

  // Set levelChange_flag if huge change in level
  if ( abs(USsample - lastUSsample) > USlevelChangeThreshold )
  {
    levelChange_flag = true;
    Serial.println("Level change!");
  }

  // Store in cyclic buffer
  storeVal(us, USsample);
  lastUSsample = USsample;

  // Print average
  Serial.print("US avg "); Serial.println(USsample);

  // Serial.println("Ultrasonic END");
  Serial.println();
}


void sampleHumidityAndTemperature()
{
  // Serial.println("Hum and temp BEGIN");

  DHT22_ERROR_t errorCode;
  errorCode = myDHT22.readData();
  switch(errorCode)
  {
    case DHT_ERROR_NONE:
      // t1 = myDHT22.getTemperatureCInt();
      // h1 = myDHT22.getHumidityInt();
      storeVal( t1, myDHT22.getTemperatureCInt() );
      storeVal( h1 , myDHT22.getHumidityInt() );
      Serial.print("Temp "); Serial.print(t1[numSampleBuffer-1]); Serial.println(" /10 C");
      Serial.print("Hum "); Serial.print(h1[numSampleBuffer-1]); Serial.println(" /10 %");
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

  // Serial.println("Hum and temp END");
  Serial.println();
}


void sampleBattery()
{
  // Serial.println("Battery BEGIN");

  int batteryReading = analogRead(batteryPin);
  Serial.print("Batt "); Serial.println(batteryReading);
  float batteryVoltage = float(batteryReading)*10.0/1023.0;
  Serial.print(batteryVoltage); Serial.println(" Volts");

  // Store value for sending to server
  // b = batteryReading;
  storeVal( b , batteryReading );

  // Serial.println("Battery END");
  Serial.println();
}


void sendData()
{
  Serial.println("Transmit BEGIN");

  // Switch to modem within SoftwareSerial
  modemSerial.listen();

  // Start timing how long modem is on
  long sendDataTimeStart = millis();

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
        sampleSignalStrength();
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
      Serial.println("Can't connect");
    }

    // Power down modem
    powerDown();
  }
  else
  {
    Serial.println("Couldn't powerUp modem");
  }

  // Stop stopwatch on sending data
  sendDataTimer = millis() - sendDataTimeStart;

  Serial.println("Transmit END");
  Serial.println();
}




//                                             //
//               MODEM FUNCTIONS               //
//                                             //
void attemptSendHTTPdata()
{
  Serial.println("Send HTTP");

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
    Serial.println("Weird bearer, break");
    return;
  }

  // Start HTTP session
  modemSerial.write("AT+HTTPINIT\r");
  findOK();

  // Set HTTP URL and load data into GET request
  delay(500);
  createAndPushHTTPgetString();
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

  // Clear sample buffers after successful send
  if ( HTTPcode == 200 )
  {
    emptyDataStorage();
    numSamplesSinceLastSendData = 0;
  }

  // return true;
}


boolean isConnected()
{
  // Serial.println(); Serial.println("isConnected()");
  modemSerial.write("AT+CREG?\r\n");

  finder.find("+CREG: ");  // Set CREG=1 to return unsolicited network registration code

  // Get network status, could be values 0, 2 (not connected) OR 1 (connected)
  long response = finder.getValue(',');

  // Make sure it didn't time out
  findOK();

  // Test and display signal strength while trying to connect
  // sampleSignalStrength();

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

void sampleSignalStrength()
{
  modemSerial.write("AT+CSQ\r\n");
  finder.find("+CSQ: ");
  long rssi = finder.getValue();
  findOK();
  Serial.print("CSQ "); Serial.println(rssi);
  csq = int(rssi);
}

boolean powerUp()
{
  Serial.println("powerUp()");
  toggleModemPower();

  finder.find("\r\n"); // First return

  // char debugByte = 255; // FF
  while ( finder.find("\r\n") == false ) {}
  // Serial.println("nulls skipped");

  finder.find("\r\n"); // Last return
  // for ( int i=0; i < 9; i++ ) finder.find("."); // Units 1.6, 1.10
  // for ( int i=0; i < 1; i++ ) finder.find("."); // Units 1.2, 1.3, 1.7, 1.8

  // Call Ready can take 10 seconds
  if (finder.find("Call Ready"))
  {
    // Successfully turned on
    Serial.println("Modem responded");
    return true;
  }
  else
  {
    Serial.println("No modem response");
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
  delay(2000);
  digitalWrite(powerPin,LOW);
}


void findOK()
{
  if (!finder.find("OK")) Serial.println("OK not found!");
}



//                                             //
//           ULTRASONIC FUNCTIONS              //
//                                             //

#ifdef I2Cultrasonic_flag
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
#endif




//                                             //
//               UTILITY FUNCTIONS             //
//                                             //
void setArray(int* _array, int _value)
{
  for (int entry=0; entry < numSampleBuffer; entry++) _array[entry] = _value;
}

void storeVal(int* _array, int _value)
{
  for (int entry=0; entry < numSampleBuffer-1; entry++) _array[entry] = _array[entry+1];
  _array[numSampleBuffer-1] = _value;
}

void createAndPushHTTPgetString()
{
    // Create HTTP GET string - OPTIMIZE, maybe with replace string?
  String postData = "AT+HTTPPARA=\"URL\",\"http://compology.herokuapp.com/bigreceive?id=";
  postData += id;
  postData += "&";
  postData.toCharArray(string2charBuffer, bufferSize);
  modemSerial.write(string2charBuffer);

  postData = "us=";
  for (int sample=0; sample < numSampleBuffer; sample++)
  {
    postData += us[sample];
    if (sample == numSampleBuffer-1) postData += "&";
    else postData += "-";
  }
  postData.toCharArray(string2charBuffer, bufferSize);
  modemSerial.write(string2charBuffer);

  postData = "ir=";
  for (int sample=0; sample < numSampleBuffer; sample++)
  {
    postData += ir[sample];
    if (sample == numSampleBuffer-1) postData += "&";
    else postData += "-";
  }
  postData.toCharArray(string2charBuffer, bufferSize);
  modemSerial.write(string2charBuffer);

  postData = "t1=";
  for (int sample=0; sample < numSampleBuffer; sample++)
  {
    postData += t1[sample];
    if (sample == numSampleBuffer-1) postData += "&";
    else postData += "-";
  }
  postData.toCharArray(string2charBuffer, bufferSize);
  modemSerial.write(string2charBuffer);

  postData = "h1=";
  for (int sample=0; sample < numSampleBuffer; sample++)
  {
    postData += h1[sample];
    if (sample == numSampleBuffer-1) postData += "&";
    else postData += "-";
  }
  postData.toCharArray(string2charBuffer, bufferSize);
  modemSerial.write(string2charBuffer);

  postData = "t2=";
  for (int sample=0; sample < numSampleBuffer; sample++)
  {
    postData += t2[sample];
    if (sample == numSampleBuffer-1) postData += "&";
    else postData += "-";
  }
  postData.toCharArray(string2charBuffer, bufferSize);
  modemSerial.write(string2charBuffer);

  postData = "h2=";
  for (int sample=0; sample < numSampleBuffer; sample++)
  {
    postData += h2[sample];
    if (sample == numSampleBuffer-1) postData += "&";
    else postData += "-";
  }
  postData.toCharArray(string2charBuffer, bufferSize);
  modemSerial.write(string2charBuffer);

  // Signal quality data
  postData = "sq=";
  postData += csq;
  postData += "&";
  postData += "tm=";
  postData += sendDataTimer/1000;
  postData += "&";
  postData.toCharArray(string2charBuffer, bufferSize);
  modemSerial.write(string2charBuffer);

  postData = "b=";
  for (int sample=0; sample < numSampleBuffer; sample++)
  {
    postData += b[sample];
    if (sample == numSampleBuffer-1) postData += "&";
    else postData += "-";
  }
  postData += "\"\r";
  postData.toCharArray(string2charBuffer, bufferSize);
  modemSerial.write(string2charBuffer);
}
