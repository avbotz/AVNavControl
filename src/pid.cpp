#include "pid.h"

Kalman pitchK(PITCH_BIAS);
Kalman rollK(ROLL_BIAS);

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

	pitchPID->setBounds(-30,30); //basically if ur more than 45 degrees off dont overcompensate too  much
	headingPID->setBounds(-180,180); //same as above
	depthPID->setBounds(-6,6); //6 inches

	//initially we are just trying to stay at the initial state

	pitchPID->setSetpoint(0.0f);
	headingPID->setSetpoint(0.0f);
	depthPID->setSetpoint(0.0f);
	
	//we scale so that we dont have to after calculating pid
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
	fAccX = (fAccX + 2 * accX) / 3;
	fAccX = (fAccX + 2 * accX) / 3;
	fGyrZ = (fGyrZ + 2 * gyrZ) / 3;

	//based on the orientation of the sensors, tan(pitch) = y/z and tan(roll) = x/z
	//im not too sure why daniel calculated roll because we have no way of correcting for it

	accP = atan2((fAccY - MU_Y_ACC), (fAccZ - MU_Z_ACC)) * 57.3f; //57.3 is 180/PI, converting from radians to degrees
	accR = atan2((fAccX - MU_X_ACC), (fAccZ - MU_Z_ACC)) * 57.3f;

	//use the kalman filters on pitch and roll and just adjust heading
	calcP = pitchK.calculate(gyrX, accP);
	calcR = rollK.calculate(gyrY, accR);
	//because positive is CCW for the IMU and we treat it as positive with motors
	calcH += (MU_Z_GYR - fGyrZ) * GYR_SCALE * DT;

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
	if ( (fabs(dHA - calcH) < fabs(dHB - calcH)) && (fabs(dHA - calcH) < fabs(dHS - calcH)) ) {	 //makes sure its using the smallest interval to get
		dHC = dHA;																				  //to the desired heading so going from 2  359 goes
	}																							   //a few degrees CCW instead of going all
	else if (fabs(dHB - calcH) < fabs(dHS - calcH)) {											   //the way around CW
		dHC = dHB;
	}
	else {
		dHC = dHA;
	}

	//dont attempt to correct heading if less than 5 degrees off, just move
	if (fabs(dHC-calcH) < 5) {
		isTurn = false;
		isMove = true;
	}
	//attempt to correct heading while moving if between 5 and 15 degrees off
	else if (fabs(dHC-calcH) < 15) {
		isTurn = true;
		isMove = true;
	}
	//if too far off then dont move, just turn
	else {
		isTurn = true;
		isMove = false;
	}
	float headError = dHC - calcH;
	
	headingPID->setProcessValue(headError);
	
	float depthError = desDepth - depth;
	depthPID->setProcessValue(depthError);
	//always correct for pitch
	isPitch = true;
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

	//desPower is between 0 and 200 i think where 0 is full backward and 200 is full forward
	forwardPower = (100 - desPower) * 0.01f * (1 - fabs(hpid));
	pitchPower = ppid * (1 - fabs(dpid));
	//right motor is more powerful than left, back motor is runs in reverse of the others
	if (isMove && isTurn) {
		motorSpeed[LEFT] = hpid + forwardPower;
		motorSpeed[RIGHT] = -hpid + forwardPower;
	}
	else if (!isMove && isTurn) {
		motorSpeed[LEFT] = hpid;
		motorSpeed[RIGHT] = -hpid;
	}
	else {  // isMove && !isTurn
		motorSpeed[LEFT] = forwardPower;
		motorSpeed[RIGHT] = forwardPower;
	}
	
	if (isPitch) {
		motorSpeed[FRONT] = dpid - pitchPower;
		motorSpeed[BACK] = (-1.0f * dpid) - pitchPower;
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

	motorArray[LEFT] = powerNum[LEFT];
	motorArray[RIGHT] = powerNum[RIGHT];
	motorArray[FRONT] = powerNum[FRONT];
	motorArray[BACK] = powerNum[BACK];
}


void give_data(int accx, int accy, int accz, int gyrx, int gyry, int gyrz) {

	accX = accx;
	accY = accy;
	accZ = accz;
	gyrX = gyrx;
	gyrY = gyry;
	gyrZ = gyrz;
}
