#include "pid.h"

//Average values with IMU on flat surface
float MU_X_ACC;
float MU_Y_ACC;
float MU_Z_ACC;
// Subtract 256 in the Z direction to account for gravity. According to IMU data
// sheet, the sensitivity at 2g mode (which is what we are using) is
// 256 least significant bits per 1 g. Normal Earth gravity = 1 g.

float MU_X_GYR;
float MU_Y_GYR;
float MU_Z_GYR;

float MU_X_COM;
float MU_Y_COM;
float MU_Z_COM;

Kalman pitchK;
Kalman rollK;

volatile float calcP, calcH, calcR;
float accP, accR;

float accX(0), accY(0), accZ(0), gyrX(0), gyrY(0), gyrZ(0);

float fAccX = MU_X_ACC;
float fAccY = MU_Y_ACC;
float fAccZ = MU_Z_ACC;
float fGyrZ = MU_Z_GYR;

bool isMove;
bool isTurn;
bool isPitch;

PID* pitchPID = NULL;
PID* headingPID = NULL;
PID* depthPID = NULL;

extern PC pc;

extern LocalFileSystem local;

int pid_loops_since_alive = 0;
float pressure_average = 0;

void reset_pid()
{
	pitchPID->reset();
	headingPID->reset();
	depthPID->reset();
   
	fAccX = MU_X_ACC;
	fAccY = MU_Y_ACC;
	fAccZ = MU_Z_ACC;
	fGyrZ = MU_Z_GYR;
	
	pressure_average = 0;
	
	calcP = calcH = calcR = 0;
//	pc.send_message("\n\nhello\n\n");
}

void init_pid()
{
	pitchPID = new PID();
	headingPID = new PID();
	depthPID = new PID();
	
	pitchPID->setGains(PITCH_KP, PITCH_KI, PITCH_KD);
	headingPID->setGains(HEADING_KP, HEADING_KI, HEADING_KD);
	depthPID->setGains(DEPTH_KP, DEPTH_KI, DEPTH_KD);

	pitchPID->setBounds(-30,30); //basically if you're more than 30 degrees off don't overcompensate too much
	headingPID->setBounds(-180,180); //same as above
	depthPID->setBounds(-6,6); //6 inches

	//initially we are just trying to stay at the initial state

	pitchPID->setSetpoint(0.0f);
	headingPID->setSetpoint(desHead);
	depthPID->setSetpoint(desDepth);
	
	//we scale so that we don't have to after calculating pid
	//the scales are the same as Daniel's

	headingPID->setScale(1.0f/180);
	pitchPID->setScale(100.0f/35);
	depthPID->setScale(1.0f/3);
	
	headingPID->setDt(DT);
	pitchPID->setDt(DT);
	depthPID->setDt(DT);
	
	headingPID->setBias(0.0f);
	pitchPID->setBias(0.0f);
	depthPID->setBias(0.0f);
	
	headingPID->setIntegralRegion(1.0f/6,1.0f/6); //-30 to 30 degrees
	pitchPID->setIntegralRegion(-1000.0f/35,1000.0f/35);	//-10 to 10 degrees
	depthPID->setIntegralRegion(-12.0f, 8.0f);	//-3 to 2 feet
	
	//Read from the log file
	FILE* calibration_file = fopen("/local/CALIBRAT", "r");
	fscanf(
	 calibration_file,
	 "\t%f\t%f\t%f\r\n\t%f\t%f\t%f\r\n\t%f\t%f\t%f\r\n",
	 &MU_X_ACC, &MU_Y_ACC, &MU_Z_ACC, 
	 &MU_X_GYR, &MU_Y_GYR, &MU_Z_GYR,
	 &MU_X_COM, &MU_Y_COM, &MU_Z_COM 
	);
	fclose(calibration_file);

	MU_Z_ACC -= 256.0f;

	pitchK.setBias(MU_X_GYR);
	rollK.setBias(MU_Y_GYR);

	reset_pid();
}

void do_pid()
{
	//if(debug || isAlive) return;
	//dammit adit
	float ppid, hpid, dpid;
	
	//check kill state, maybe reset
	if (isAlivePrev == false && isAlive == true)
	{
		pid_loops_since_alive = DEAD_TIME*SAMPLES_PER_SECOND/1000;
		reset_pid();
	}
	
	//check imu data
	//maybe get new compass reading
	//char* test;
	//pc.printf("%g\r\n", gyrZ);

	//Daniel's code before just averaged the values below
	// the weighted average is because if we take fAcc* to be the value in the middle of the
	// time period before and acc* is the end of the current time period, the middle of the 
	// current time period would be closer to the end of this time period
	// |--old--|--__--|new

	fAccX = (fAccX + 2 * accX) / 3;
	fAccY = (fAccY + 2 * accY) / 3;
	fAccZ = (fAccZ + 2 * accZ) / 3;
	fGyrZ = (fGyrZ + 2 * gyrZ) / 3;

	//based on the orientation of the sensors, tan(pitch) = y/z and tan(roll) = x/z

	//57.3 is 180/PI, converting from radians to degrees
	accP = atan2((fAccY - MU_Y_ACC), (fAccZ - MU_Z_ACC)) * 57.3f;
	accR = atan2((fAccX - MU_X_ACC), (fAccZ - MU_Z_ACC)) * 57.3f;
	
	//use the kalman filters on pitch and roll and just adjust heading
	calcP = pitchK.calculate(gyrX, accP);
	//calcR = rollK.calculate(gyrY, accR);	We don't have any way to fix it, why bother? :(
	//TODO: add more motors
	//because positive is CCW for the IMU and we treat it as positive with motors
	calcH += (MU_Z_GYR - fGyrZ) * GYRO_SCALE * DT;

	//360 degrees in a circle so we want our heading between 0 and 360
	if (calcH >= 360)
	{
		calcH -= 360;
	}
	else if (calcH < 0)
	{
		calcH += 360;
	}

	double dHS, dHA, dHB, dHC;	//desired heading small, actual, and big and closest which is the best of the previous 3
	dHS = desHead - 360;
	dHA = desHead;
	dHB = desHead + 360;
	
	/*
	 * This block of code ensures that we use the smallest interval to get to the desired heading.
	 * For example, going from h=2 to h=359 moves CCW 3 degrees instead of CW 357 degrees.
	 */
	if ( (fabs(dHA - calcH) < fabs(dHB - calcH)) && (fabs(dHA - calcH) < fabs(dHS - calcH)) )
	{	
		dHC = dHA;																				  
	}																							   
	else if (fabs(dHB - calcH) < fabs(dHS - calcH))
	{	
		dHC = dHB;
	}
	else
	{
		dHC = dHS;
	}
/*
	//don't attempt to correct heading if less than 5 degrees off, just move
	if (fabs(dHC-calcH) < 5)
	{
		isTurn = false;
		isMove = true;
	}
	//attempt to correct heading while moving if between 5 and 15 degrees off
	else if (fabs(dHC-calcH) < 10)
	{
		isTurn = true;
		isMove = true;
	}
	*/
	isTurn = true;
	isMove = fabs(dHC-calcH) < 15;
	//if too far off then don't move, just turn
	/*
	else
	{
		isTurn = true;
		isMove = false;
	}
	*/
	//always correct for pitch
	isPitch = true;
	
	float headError = dHC - calcH;
	float depthError = desDepth - depth;
	
	hpid = headingPID->update(calcH, dHC);
	dpid = depthPID->update(depth, desDepth);
	ppid = pitchPID->update(calcP, 0.0f);
	
	update_motors(hpid, dpid, ppid);
}

void get_compass()
{

}

void update_motors(float hpid, float dpid, float ppid)
{
	float motorSpeed[4];
	float forwardPower, pitchPower;

	//desPower is between 0 and 200, 0 is full speed backwards, extrapolate.
//	hpid = (hpid < -1) ? -1 : ((hpid > 1) ? 1 : hpid);
//	dpid = (dpid < -1) ? -1 : ((dpid > 1) ? 1 : dpid);
//	ppid = (ppid < -1) ? -1 : ((ppid > 1) ? 1 : ppid);
//	dpid = 0;
	forwardPower = (100 - desPower) * 0.02f * (1/(fabs(3*hpid)+1));//(1 - fabs(hpid));
	pitchPower = ppid * (1/(fabs(dpid/4)+1));
	//right motor is more powerful than left, back motor is runs in reverse of the others
	if (isMove && isTurn)
	{
		motorSpeed[LEFT] = hpid + forwardPower;
		motorSpeed[RIGHT] = -hpid + forwardPower;
	}
	else if (isTurn)
	{
		motorSpeed[LEFT] = hpid;
		motorSpeed[RIGHT] = -hpid;
	}
	else
	{  // isMove && !isTurn
		motorSpeed[LEFT] = forwardPower;
		motorSpeed[RIGHT] = forwardPower;
	}
	
	if (isPitch)
	{
		//signs probably arent correct
		motorSpeed[FRONT] = dpid - pitchPower;
		motorSpeed[BACK] = dpid +  pitchPower;
	}
	//should never be used cuz assume always pitched
	else
	{
		motorSpeed[FRONT] = dpid;
		motorSpeed[BACK] = dpid;
	}
	
	int powerNum[4];
	motorSpeed[RIGHT] *= RIGHT_MULTIPLIER;
	motorSpeed[FRONT] *= FRONT_MULTIPLIER;
	motorSpeed[LEFT] *= LEFT_MULTIPLIER;
	motorSpeed[BACK] *= BACK_MULTIPLIER;
	//motorSpeed[FRONT] *= .7;	//the front motor is new
	//motorSpeed is a number around zero so the following scales them 0 - 254 which the motors require
	for (int i = 0; i < 4; i++)
	{
		powerNum[i] = motorSpeed[i] * 56;
		if (powerNum[i] < 0)
		{
			powerNum[i] += 106;
			if (powerNum[i] < 0) powerNum[i] = 0;
		}
		else if (powerNum[i] > 0)
		{
			powerNum[i] += 148;
			if (powerNum[i] > 254) powerNum[i] = 254;
		}
		else
		{
			powerNum[i] = 127;
		}
	}
	
	if (pid_loops_since_alive)
	{
		if (pid_loops_since_alive % (SAMPLES_PER_SECOND/10) == 0)
		{
			pressure_average += pressure.getValueCalibrated();
		}
		
		if (pid_loops_since_alive == 1)
		{
			pressure_average /= (DEAD_TIME/100);
			float new_b = pressure.b - pressure_average;
			pressure.changeB(new_b);
		}
			
		--pid_loops_since_alive;
		motorArray[LEFT] = 127;
		motorArray[RIGHT] = 127;
		motorArray[FRONT] = 127;
		motorArray[BACK] = 127;
		calcH = 0.0f;		
	}
	
	else
	{
	// If the sub is dead, then turn the motors off. Otherwise, set them to
	// the values that came out of PID.
	// Note: When the sub is dead, the kill switch actually cuts power to the
	// motors, so they stop moving. This is here so that the motors don't start
	// moving when the sub is unkilled (alive) until we want them to.
		motorArray[LEFT] = isAlive ? powerNum[LEFT] : 127;
		motorArray[RIGHT] = isAlive ? powerNum[RIGHT] : 127;
		motorArray[FRONT] = isAlive ? powerNum[FRONT] : 127;
		motorArray[BACK] = isAlive ? powerNum[BACK] : 127;
	}
	
}

void give_data(int accx, int accy, int accz, int gyrx, int gyry, int gyrz)
{
	accX = accx;
	accY = accy;
	accZ = accz;
	gyrX = gyrx;
	gyrY = gyry;
	gyrZ = gyrz;
}
