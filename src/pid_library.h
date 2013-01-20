// "PID without a PhD" explains PID quite well: 
// <http://www.eetimes.com/design/embedded/4211211/PID-without-a-PhD>
// Look at the PDF version linked from that page if you want pictures with your
// words.

class PID
{
public:
	//adjustments order: bias->bounds->scale

	void setGains(float kp, float ki, float kd);
	void setBounds(float inputMax, float inputMin);
	void setScale(float inputScale);
	void setBias(float bias);
	void setDt(float dt);
	void setMaxI(float m);
	
	void setSetpoint(float setpoint);
	void setProcessValue(float processValue);
	float calculate();
	float update(float processValue, float setpoint);
	void reset();
	void scale_input();
	
	//the gains for PID
	float _kp, _ki, _kd;

private:
	float calculate_i();
	float calculate_d();

	//the max and min outside which the integral term will not be adjusted
	//this is to avoid integral windup where the integral term increases
	//too fast because the process value is too far off the setpoint
	//which would cause overshooting
	float _inputMax, _inputMin;

	//allows for input scaling to be built in to PID
	float _inputScale;

	//if needed for some reason
	float _bias;

	//the desired value
	float _setpoint;

	//the current process value and the one before
	// Process value is the current state of the system. error = difference between process and setpoint
	float _processValue, _last;

	//the current integral term
	float _integral;
	
	
	float _dt;
	
	float _scaledInput;
	
	//maximum integral term
	float _integral_max;
};
