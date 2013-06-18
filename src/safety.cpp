#include "safety.h"

volatile bool hasLeak = false, isLowVoltage = false;
extern volatile bool isAlive;

bool needStop()
{
	return (!isAlive | hasLeak | isLowVoltage);
}