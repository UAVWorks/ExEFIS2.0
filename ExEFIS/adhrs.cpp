#include "adhrs.h"
#include "HRS_9250.h"
#include "hsc_pressure.h"
#include <QApplication>
#include <QFile>
#include <QByteArray>
#include <QString>
#include <math.h>
#include <QDebug>
#include <QTimer>
#include <QWidget>
#include "wiringPi.h"




adhrs::adhrs()
{
	char caldata[22];
	staticpress = new hsc_pressure();
	staticpress->set_params(15, 0);
	airspeed = new hsc_pressure(1);
	airspeed->set_params(1, -1);

//	float gyroBias[3];
//	gyroBias[0] = 49.374046;
//	gyroBias[1] = 0.229008;
//	gyroBias[2] = 78.167938;	
//	
//	float accelBias[3];
//	accelBias[0]  = 177.612305;
//	accelBias[1] = 14.709473;
//	accelBias[2] = 62.500000;
//	
//	float magBias[3];
//	magBias[0] = 224.639008;
//	magBias[1] = 178.052505;
//	magBias[2] = -220.621109;
//	
//	float magScale[3];
//	magScale[0] = 0.983562;
//	magScale[1] = 1.071642;
//	magScale[2] = 0.952255;
	
	//current kitfox values
	
		float gyroBias[3];
	gyroBias[0] = -39.053436;
	gyroBias[1] = 0.251908;
	gyroBias[2] = 148.519089;
	
	float accelBias[3];
	accelBias[0]  = 480.000;
	accelBias[1] = 14.282227;
	accelBias[2] = 218.750000;
	
	float magBias[3];
	magBias[0] = -70.752441;
	magBias[1] = 252.002869;
	magBias[2] = 252.002869;
	
	float magScale[3];
	magScale[0] = 1.018518;
	magScale[1] = 0.989418;
	magScale[2] = 0.992569;
	
//	X - Axis sensitivity adjustment value + 1.18
//Y - Axis sensitivity adjustment value + 1.19
//Z - Axis sensitivity adjustment value + 1.14
	
	//float* ppGyroBias, float* ppAccelBias, float* ppMagBias, float* ppMagScale
	HRS_9250 *hrs = new HRS_9250(gyroBias, accelBias, magBias, magScale);
	//HRS_9250 *hrs = new HRS_9250;
	int s = hrs->Init(false, false, false);
	
	//bno055 = new BNO055(BNO055_ID, BNO055_ADDRESS_A);
	//bno055->begin(BNO055::OPERATION_MODE_NDOF);
	
	bool cal = false;
	QFile *calfile = new QFile("/home/pi/sensorcal.txt");
	if (calfile->exists())
	{
		if (calfile->open(QIODevice::ReadOnly))
		{
			while (!calfile->atEnd()) {
				QByteArray line = calfile->readLine();
				calfile_process_line(line, caldata);
			}
			/* do we calibrate?*/
			cal = calfile_validate(caldata);
			if (cal) qDebug() <<"found valid caldata" << calfile->fileName();
		}
	}	

	//bno055->begin(cal, BNO055::OPERATION_MODE_NDOF, caldata); //used to be IMUPLUS
	
//	if (!bno055->isFullyCalibrated())
//	{
		//while (1)
			;
//	}
}


adhrs::~adhrs()
{
}


void adhrs::readAll(void)
{	
	//imu::Vector<3> v = bno055->getVector(BNO055::adafruit_vector_type_t::VECTOR_EULER);	
	int error = 1;
	int retry = 0;
	while (error && retry < 3)
	{		
		error = 0;
		//imu::Quaternion q = bno055->getQuat(&error);
		//imu::Vector<3> v = hrs->GetEuler(&error);
		if (!error)
		{	
			//imu::Vector<3> v = q.toEuler();
			this->euHeading = hrs->getHeading(); 
			;    //page 35 in BNO055 manual for order here
			this->euRoll = hrs->getRoll();
			this->euPitch = hrs->getPitch();
			staticPressurePSI = staticpress->getPressure();
			aspPressureMBAR = airspeed->getPressure();
			imu::Vector<3> a = hrs->GetAccelerometer(&error);
			slipRAW = -2.0f*(a.y());	
		}
		retry++;
	}
	if (error)
	{		
		qDebug() << "Read Error - 3 retrys failed" << QString::number(error, 10) << ","; 
	}
	
	if (std::isnan(this->euHeading))
	{
		hrs->resetAlgorithm();
	}
}


int adhrs::getAllSixRaw(float* data)
{		
	
	int status = 0;
	data[0] = this->staticPressurePSI;
	data[1] = this->aspPressureMBAR;
	data[2] = (this->euHeading * (180.0f / M_PI)); //quat is in radians
	data[3] = (this->euRoll * (180.0f / M_PI)); //quat is in radians
	data[4] = (this->euPitch * (180.0f / M_PI)); //quat is in radians
	data[5] = (this->slipRAW);
	
	return status;
}

int adhrs::getOffsets(char* calData)
{
	if (true) //get offsets)
	{
		
		return 1;
	}
	return 0;
}


int adhrs::setOffsets(char* calData)
{
	//set offsets
	return 1;
}


void adhrs::getCalibration(char* cal)
{	
	//get calibration
}


void adhrs::calfile_process_line(QByteArray &line, char* data)
{
	short* dat = (short*)data;
	if (line.startsWith("Accel Offset X"))
	{		
		short val = line.split('"')[1].toShort();
		dat[0] = val;
	}
	if (line.startsWith("Accel Offset Y"))
	{		
		short val = line.split('"')[1].toShort();
		dat[1] = val;
	}
	if (line.startsWith("Accel Offset Z"))
	{		
		short val = line.split('"')[1].toShort();
		dat[2] = val;
	}
	if (line.startsWith("Gyro Offset X"))
	{		
		short val = line.split('"')[1].toShort();
		dat[3] = val;
	}
	if (line.startsWith("Gyro Offset Y"))
	{		
		short val = line.split('"')[1].toShort();
		dat[4] = val;
	}
	if (line.startsWith("Gyro Offset Z"))
	{		
		short val = line.split('"')[1].toShort();
		dat[5] = val;
	}
	if (line.startsWith("Mag Offset X"))
	{		
		short val = line.split('"')[1].toShort();
		dat[6] = val;
	}
	if (line.startsWith("Mag Offset Y"))
	{		
		short val = line.split('"')[1].toShort();
		dat[7] = val;
	}
	if (line.startsWith("Mag Offset Z"))
	{		
		short val = line.split('"')[1].toShort();
		dat[8] = val;
	}
	if (line.startsWith("Accel Radius"))
	{		
		short val = line.split('"')[1].toShort();
		dat[9] = val;
	}
	if (line.startsWith("Mag Radius"))
	{		
		short val = line.split('"')[1].toShort();
		dat[9] = val;
	}
}


bool adhrs::calfile_validate(char* data)
{
	return (true);
}

