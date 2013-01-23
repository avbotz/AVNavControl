#include "pid_library.h"

void PID::setGains(float kp, float ki, float kd)
{
	_kp = kp;
	_ki = ki;
	_kd = kd;
}

void PID::setBounds(float inputMin, float inputMax)
{
	_inputMin = inputMin;
	_inputMax = inputMax;
}

void PID::setScale(float inputScale)
{
	_inputScale = inputScale;
}

void PID::setBias(float bias)
{
	_bias = bias;
}

void PID::setDt(float dt)
{
	_dt = dt;
}

void PID::setSetpoint(float setpoint)
{
	_setpoint = setpoint;
}

void PID::setProcessValue(float processValue)
{
	_last = _scaledInput;
	_processValue = processValue;
}

float PID::calculate()
{
	scale_input();
	return (_kp * _scaledInput + _ki * calculate_i() + _kd * calculate_d());
}

void PID::reset()
{
	_setpoint = 0.0f;
	_processValue = _last = 0.0f;
	_integral = 0.0f;
}

float PID::calculate_i()
{

	if ((_last - _setpoint) * (_processValue - _setpoint) < 0)
	{
		_integral = 0.0f;
	}
	else
	{
		_integral += _scaledInput * _dt;
	}
	return _integral;
}

float PID::calculate_d()
{
	return (_scaledInput - _last) / _dt;
}

float PID::update(float processValue, float setpoint)
{
	setSetpoint(setpoint);
	setProcessValue(processValue);
	calculate();
}

void PID::scale_input()
{
	if (_processValue + _bias - _setpoint > _inputMax) {
		_scaledInput = (_inputMax) * _inputScale;
	}

	else if (_processValue + _bias - _setpoint < _inputMin) {
		_scaledInput = (_inputMin) * _inputScale;
	}
	else {
		_scaledInput = (_processValue + _bias - _setpoint) * _inputScale;
	}
}
