// **** INCLUDES *****
#include "LowPower.h"

//long time;

void setup()
{
    // No setup is required for this library
    Serial.begin(9600);
    Serial.println("Periodic sleep sketch BEGIN");
    delay(100);
}

void loop() 
{
    // Enter power down state for 8 s with ADC and BOD module disabled
//    time = millis();
//    Serial.print("Time beginning loop "); Serial.println(time);
//    delay(100);
    LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
//    LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
//    Serial.print("Time after loop "); Serial.println(millis()-time);
//    Serial.println("Sleep interrupt");
//    delay(100);
    
    // Do something here
    // Example: Read sensor, data logging, data transmission.
}
