#define I2Cultrasonic_flag


#ifdef I2Cultrasonic_flag

#include <LowPower.h>

#endif


void setup()
{
	
}

void loop()
{
	#ifdef I2Cultrasonic_flag
	LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
	#endif
}