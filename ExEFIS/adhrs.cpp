#include "adhrs.h"
#include "BNO055.h"
#include "hsc_pressure.h"
#include <QApplication>
#include <QFile>
#include <QByteArray>
#include <QString>
#include <math.h>
#include <QDebug>




adhrs::adhrs()
{
	char caldata[22];
	staticpress = new hsc_pressure();
	airspeed = new hsc_pressure(1);
	airspeed->set_params(5, -5);
	bno055 = new BNO055(BNO055_ID, BNO055_ADDRESS_A);
	//bno055->begin(BNO055::OPERATION_MODE_NDOF);
	qDebug("BNO055 Mode is MODE_NDOF");
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
			if (cal) qDebug() <<"BNO055 found valid caldata" << calfile->fileName();
		}
	}
	bno055->begin(cal, BNO055::OPERATION_MODE_NDOF, caldata); //used to be IMUPLUS
	
	if (!bno055->isFullyCalibrated())
	{
		//while (1)
			;
	}
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
		imu::Quaternion q = bno055->getQuat(&error);
		if (!error)
		{	
			imu::Vector<3> v = q.toEuler();
			this->euHeading = v[0];    //page 35 in BNO055 manual for order here
			this->euRoll = v[2];
			this->euPitch = v[1];
			staticPressurePSI = staticpress->getPressure();
			aspPressureMBAR = airspeed->getPressure();
			imu::Vector<3> a = bno055->getVector(BNO055::adafruit_vector_type_t::VECTOR_ACCELEROMETER);
			slipRAW = a.y();	
		}
		retry++;
	}
	if (error)
	{		
		qDebug() << "BNO055 Read Error - 3 retrys failed" << QString::number(error, 10) << ","; 
	}
}


int adhrs::getAllSixRaw(float* data)
{
	int status = 0;
	data[0] = this->staticPressurePSI;
	data[1] = this->aspPressureMBAR;
	data[2] = 360 - (180 + this->euHeading * (180.0f / M_PI)); //quat is in radians
	data[3] = this->euRoll * (180.0f / M_PI); //quat is in radians
	data[4] = this->euPitch * (180.0f / M_PI); //quat is in radians
	data[5] = this->slipRAW;
	
	return status;
}


uint adhrs::readBNORegister(uint reg)
{
	return(this->bno055->readRegister(reg));
}


int adhrs::writeBNORegister(uint reg, uint value)
{
	this->bno055->writeRegister(reg, value);
	return (1);
}

int adhrs::getOffsets(char* calData)
{
	if (this->bno055->getSensorOffsets(calData))
	{
		
		return 1;
	}
	return 0;
}


int adhrs::setOffsets(char* calData)
{
	this->bno055->setSensorOffsets(calData);
	return 1;
}


void adhrs::getCalibration(char* cal)
{	
	this->bno055->getCalibration(&cal[0], &cal[1], &cal[2], &cal[3]);
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
