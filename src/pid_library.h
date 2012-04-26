
class PID
{
public:
    void setGains(float kp, float ki, float kd);
    void setBounds(float inputMax, float inputMin);
    void setScale(float inputScale);
    void setBias(float bias);
    void setDt(float dt);
    
    void setSetpoint(float setpoint);
    void setProcessValue(float processValue);
    float calculate();
    void reset();
    void scale_input();
private:

    float calculate_p();
    float calculate_i();
    float calculate_d();

    //the gains for PID
    float _kp, _ki, _kd;

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

    //the current process value, last, and second last
    // Process value is the current state of the system. error = difference between process and setpoint
    float _processValue, _last, _last2;

    //the current integral term
    float _integral;
    
    float _dt;
    
    float _scaledInput;
};
