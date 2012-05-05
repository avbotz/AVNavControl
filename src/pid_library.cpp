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
	_last = _processValue;
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
	// only if the term is outside the bounds, then just add the bound its outside of
	// instead of adding the actual value which would throw off the integral term like crazy

	if (_processValue > _inputMax) {
		_integral += _inputMax * _dt;
	}
	else if (_processValue < _inputMin) {
		_integral += _inputMin * _dt;
	}
	else {
		_integral += _scaledInput * _dt;
	}

	return _integral;
}
float PID::calculate_d()
{
	return (_scaledInput - _last) / _dt;
}

void PID::scale_input()
{
	if (_processValue > _inputMax) {
		_scaledInput = (_inputMax + _bias) * _inputScale;
	}

	else if (_processValue + _bias < _inputMin) {
		_scaledInput = (_inputMin + _bias) * _inputScale;
	}
	else {
		_scaledInput = (_processValue + _bias) * _inputScale;
	}
}
