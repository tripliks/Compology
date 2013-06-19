byte batteryPin = A0;

void setup() 
{
	Serial.begin(9600);
	pinMode(batteryPin, INPUT);
}

void loop() 
{
	int battery = analogRead(batteryPin);
	Serial.println(battery);
}