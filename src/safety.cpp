#include "safety.h"

volatile bool hasLeak = false, isLowVoltage = false;
extern volatile bool isAlive;

enum EmergencyTypes {
	NO_EMERGENCY,
	LEAK,
	VOLTAGE_LOW,
	VOLTAGE_HIGH	// not used
};

bool needStop()
{
	return (!isAlive | hasLeak | isLowVoltage);
}

int getEmergencyType()
{
	if (hasLeak)
	{
		return LEAK;
	}
	else if (isLowVoltage)
	{
		return VOLTAGE_LOW;
	}
	else
	{
		return NO_EMERGENCY;
	}
}