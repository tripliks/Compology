#include <SoftwareSerial.h>
#include <TextFinder.h>

// SoftSerial ports connected to shield pins 7, 8 
SoftwareSerial modemSerial(7, 8);
#define powerPin 9

// Feed TextFinder the stream, 2 second timeout for searches
TextFinder  finder(modemSerial, 2); 


void setup()
{
 	modemSerial.begin(9600);               // the modemSerial baud rate   
 	Serial.begin(19200);             // the Serial port of Arduino baud rate.
 	Serial.println("GSM comm BEGIN");
}


void loop()
{

}



void sendData()
{
	// Check power, power up if necessary
	powerUp();

	// Check connection, connect if necessary, limit time of attempts
	while (!isConnected && timeConnectAttempt < timeConnectTimeout) {}

	// Send data if possible
	if (isConnected)
	{
		// Send HTTP data, wait for 200 response
		// Limit # attempts
		attemptSendHTTPdata();
	}
	else  // Didn't connect
	{
		Serial.println("Couldn't connect");
	}

	// Power down modem
	powerDown();
}


void attemptSendHTTPdata()
{
	Serial.println("attemptSendHTTPdata()");

	// Open data bearer
	modemSerial.write("AT+SAPBR=1,1\r");
	// Make sure it didn't time out
	if (finder.find("OK"))	Serial.println("OK not found!");

	// Terminate existing HTTP session??
	modemSerial.write("AT+HTTPTERM\r");
	// Responds with OK if already started, ERROR if none started yet
	delay(1000);

	// Start HTTP session
	modemSerial.write("AT+HTTPINIT\r");
	if (!finder.find("OK"))	Serial.println("OK not found!");

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
	postData += "&b=";
	postData += b;
	postData += "&\"\r";

	postData.toCharArray(string2charBuffer, 200);

	modemSerial.write(string2charBuffer);
	Serial.println(string2charBuffer);
	// Look for OK response
	if (!finder.find("OK"))	Serial.println("OK not found!");

	// Set HTTP bearer
	modemSerial.write("AT+HTTPPARA=\"CID\",1\r");
	if (!finder.find("OK"))	Serial.println("OK not found!");

	// Initiate GET request
	modemSerial.write("AT+HTTPACTION=0\r");
	if (!finder.find("OK"))	Serial.println("OK not found!");

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
	Serial.write("AT+CREG?\r\n");
	finder.find("+CREG: ");  // Set CREG=1 to return unsolicited network registration code

	// Get network status, could be values 0, 2 (not connecteod) OR 1 (connected)
	long response = finder.getValue(',');

	// Make sure it didn't time out
	if (finder.find("OK"))
	{
		// Serial.println("OK found!");
	}
	else // signals that modem timed out or something wrong with comm to it
	{
		Serial.println("OK not found!");
	}

	if (response==1) return true;
	else return false;
}


void powerUp()
{
	Serial.println("powerUp()");
	toggleModemPower();
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

void powerDown()
{
	toggleModemPower();
}

void toggleModemPower()
{
	digitalWrite(9,HIGH);
 	delay(1500);
 	digitalWrite(9,LOW);
}




void twoWayTalk_modemToSerial()
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