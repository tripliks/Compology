void setup()
{
  pinMode(9, OUTPUT);
  Serial.begin(9600);
  
  Serial.println("Toggle modem power");
}

void loop()
{
  // Toggle modem power
  digitalWrite(9,HIGH);
  delay(1500);
  digitalWrite(9,LOW);
  Serial.println("Toggle");
  
  delay(3000);
}


