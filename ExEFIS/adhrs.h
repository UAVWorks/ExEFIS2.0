#pragma once
#include "hsc_pressure.h"
#include <QByteArray>
#include <QTimer>
#include "HRS_9250.h"

class adhrs
{
public:
	adhrs();
	~adhrs();
	void readAll(void);
	int getAllSixRaw(float* data);	
	int getOffsets(char* calData);
	int setOffsets(char* calData);
	void getCalibration(char* cal);
	
	
private:
	
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
	HRS_9250 *hrs;
	
};

