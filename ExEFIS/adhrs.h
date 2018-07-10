#pragma once
#include "BNO055.h"
#include "hsc_pressure.h"
#include <QByteArray>

class adhrs
{
public:
	adhrs();
	~adhrs();
	void readAll(void);
	int getAllSixRaw(float* data);
	uint readBNORegister(uint reg);
	int writeBNORegister(uint reg, uint value);
	int getOffsets(char* calData);
	int setOffsets(char* calData);
	void getCalibration(char* cal);
	
private:
	BNO055 *bno055;
	hsc_pressure *staticpress;
	hsc_pressure *airspeed;
	float staticPressurePSI;
	float aspPressureMBAR;
	float euHeading; // page 35 in BNO055 manual, these are Euler Angles in order
	float euRoll;	// page 35 in BNO055 manual, these are Euler Angles in order
	float euPitch;	// page 35 in BNO055 manual, these are Euler Angles in order
	float slipRAW;
	void calfile_process_line(QByteArray &line, char* data);
	bool calfile_validate(char* data);
	
};

