#include "pid.h"

Kalman pitchK(MU_X_GYR);
Kalman rollK(MU_Y_GYR);

float calcP, calcH, calcR;
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

extern Serial pc;

void reset_pid() {
	pitchPID->reset();
	headingPID->reset();
	depthPID->reset();
   
	fAccX = MU_X_ACC;
	fAccY = MU_Y_ACC;
	fAccZ = MU_Z_ACC;
	fGyrZ = MU_Z_GYR;

	calcP = calcH = calcR = 0;
}

void init_pid() {
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
	
	reset_pid();
}

void do_pid() {
	//if(debug || isAlive) return;
	//dammit adit
	headingPID->setSetpoint(desHead);
	depthPID->setSetpoint(desDepth);
	float ppid, hpid, dpid;
	
	//check kill state, maybe reset
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
	if (calcH >= 360) {
		calcH -= 360;
	} else if (calcH < 0) {
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
	if ( (fabs(dHA - calcH) < fabs(dHB - calcH)) && (fabs(dHA - calcH) < fabs(dHS - calcH)) ) {	
		dHC = dHA;																				  
	}																							   
	else if (fabs(dHB - calcH) < fabs(dHS - calcH)) {	
		dHC = dHB;
	}
	else {
		dHC = dHS;
	}

	//don't attempt to correct heading if less than 5 degrees off, just move
	if (fabs(dHC-calcH) < 5) {
		isTurn = false;
		isMove = true;
	}
	//attempt to correct heading while moving if between 5 and 15 degrees off
	else if (fabs(dHC-calcH) < 15) {
		isTurn = true;
		isMove = true;
	}
	//if too far off then don't move, just turn
	else {
		isTurn = true;
		isMove = false;
	}
	//always correct for pitch
	isPitch = true;
	
	float headError = dHC - calcH;
	float depthError = desDepth - depth;
	
	headingPID->setProcessValue(headError);
	depthPID->setProcessValue(depthError);
	pitchPID->setProcessValue(calcP);
	
	hpid = headingPID->calculate();
	dpid = depthPID->calculate();
	ppid = pitchPID->calculate();
	
	update_motors(hpid, dpid, ppid);
}

void get_compass() {


}

void update_motors(float hpid, float dpid, float ppid) {
	float motorSpeed[4];
	float forwardPower, pitchPower;

	//desPower is between 0 and 200, 0 is full speed backwards, extrapolate.
	forwardPower = (100 - desPower) * 0.02f * (1 - fabs(hpid));
	pitchPower = ppid * (1 - fabs(dpid));
	//right motor is more powerful than left, back motor is runs in reverse of the others
	if (isMove && isTurn) {
		motorSpeed[LEFT] = hpid + forwardPower;
		motorSpeed[RIGHT] = -hpid + forwardPower;
	}
	else if (isTurn) {
		motorSpeed[LEFT] = hpid;
		motorSpeed[RIGHT] = -hpid;
	}
	else {  // isMove && !isTurn
		motorSpeed[LEFT] = forwardPower;
		motorSpeed[RIGHT] = forwardPower;
	}
	
	if (isPitch) {
		//signs probably arent correct
		motorSpeed[FRONT] = -dpid + pitchPower;
		motorSpeed[BACK] = -dpid - pitchPower;
	}
	//should never be used cuz assume always pitched
	else {
		motorSpeed[FRONT] = dpid;
		motorSpeed[BACK] = -1.0f * dpid;
	}
	int powerNum[4];
	motorSpeed[RIGHT] *= .85; //because the right motor is stronger
	motorSpeed[BACK] *= -1;  //because the back motor is backwards
	
	//motorSpeed is a number around zero so the following scales them 0 - 254 which the motors require
	for (int i = 0; i < 4; i++) {
		powerNum[i] = motorSpeed[i] * 56;
		if (powerNum[i] < 0) {
			powerNum[i] += 106;
			if (powerNum[i] < 0) powerNum[i] = 0;
		}
		else if (powerNum[i] > 0) {
			powerNum[i] += 148;
			if (powerNum[i] > 254) powerNum[i] = 254;
		}
		else {
			powerNum[i] = 127;
		}
	}
	
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


void give_data(int accx, int accy, int accz, int gyrx, int gyry, int gyrz) {

	accX = accx;
	accY = accy;
	accZ = accz;
	gyrX = gyrx;
	gyrY = gyry;
	gyrZ = gyrz;
}
