#pragma once
#include "Quaternion.h"
#include "Vector.h"
#include "WiringPiTransfer.h"
#include "MPU9250.h"
#include "uNavAHRS.h"

struct HRS_CAL
{
	float magx;
	float magy;
	float magz;
	float accx;
	float accy;
	float accz;
	float gyrx;
	float gyry;
	float gyrz;	
};

struct INFO_9250
{
	int id;
};


class HRS_9250
{
public:
	
	static void imuInterruptHander(void);
	
	
	HRS_9250();
	HRS_9250(float* ppGyroBias, float* ppAccelBias, float* ppMagBias, float* ppMagScale);
	~HRS_9250();
	int Init(bool doSelfTest, bool doCalibration, bool doMagCalibration);
	int GetAccelStatus(void);
	int GetMagStatus(void);
	int GetGyrStatus(void);
	int Get9250Info(INFO_9250* info);
	int SetCalibration(HRS_CAL* cal);
	int GetCalibration(HRS_CAL* cal);
	imu::Vector<3> GetEuler(int* status);
	imu::Vector<3> GetAccelerometer(int*status);
	float getRoll(void);
	float getPitch(void);
	float getHeading(void);
	/* Returns the roll angle, rad */
//	float getRoll_rad() {
//		return filter.getRoll_rad();
//	}
//
//	/* Returns the pitch angle, rad */
//	float getPitch_rad() {
//		return filter.getRoll_rad();
//	}
//
//	/* Returns the yaw angle, rad */
//	float getYaw_rad() {
//		return getYaw_rad() ;
//	}
//
//	/* Returns the heading angle, rad */
//	float getHeading_rad() {
//		return filter.getHeading_rad();
//	}
	void resetAlgorithm (void);
	
	static void RunFilter(void);
		
private:
	WiringPiI2C *mpu;
	WiringPiI2C *mag;

	

	
};



