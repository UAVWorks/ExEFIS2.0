/***************************************************************************
  This is a library for the BNO055 orientation sensor
  Designed specifically to work with the Adafruit BNO055 Breakout.
  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products
  These sensors use I2C to communicate, 2 pins are required to interface.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  Written by KTOWN for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ***************************************************************************/

#include <math.h>
#include <limits.h>
#include <unistd.h>
#include "Vector.h"
#include "BNO055.h"
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>

#include <QDebug>
#include <QString>


#define delay(x) usleep(x*1000)
int sfd;

char calData[22] = 
{ 
	0x0C,
	0x00,
	0x2B,
	0x00,
	0x14,
	0x00,
	0x91,
	0xFF,
	0xD7,
	0xFE,
	0x0C,
	0x00,
	0x03,
	0x00,
	0x00,
	0x00,
	0x01,
	0x00,
	0xE8,
	0x03,
	0x46,
	0x02
};

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_BNO055 class
*/
/**************************************************************************/

BNO055::BNO055(int sensorID, int address)
{
	_sensorID = sensorID;
	_address = address;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
/*************************************************************************/
/*!
    @brief  Sets up the HW
*/
/**************************************************************************/
bool BNO055::begin(bool calibrate, adafruit_bno055_opmode_t mode, char* cdata)
{
	//bcm2835_init();
	
	/* Enable I2C */
	//if (!bcm2835_i2c_begin()) while (1)
	//		;
	
	//bcm2835_i2c_set_baudrate(100000);
	//sfd = serialOpen("/dev/serial0", 115200);
	sfd = open("/dev/serial0", O_RDWR | O_NOCTTY);
	if (sfd == -1) while (1)
			;
	
	if (!isatty(sfd)) while (1)
			;
	
	struct termios config;
	
	if (tcgetattr(sfd, &config) >= 0)
	{
		config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                     INLCR | PARMRK | INPCK | ISTRIP | IXON);
		config.c_oflag = 0;
		config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
		config.c_cflag &= ~(CSIZE | PARENB);
		config.c_cflag |= CS8;
		config.c_cc[VMIN]  = 1;
		config.c_cc[VTIME] = 5;
		if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0) {
			while (1)
				;
		}

	}
	
	if (tcsetattr(sfd, TCSAFLUSH, &config) < 0) while (1)
			;
	

	// BNO055 clock stretches for 500us or more!
#ifdef ESP8266

	  Wire.setClockStretchLimit(1000);   // Allow for 1000us of clock stretching

#endif
	    /* Make sure we have the right device */
	char id = read8(BNO055_CHIP_ID_ADDR);
	if (id != BNO055_ID)
	{
		delay(1000);   // hold on for boot
		id = read8(BNO055_CHIP_ID_ADDR);
		if (id != BNO055_ID) {
			return false;  // still not? ok bail
		}
	}

	/* Switch to config mode (just in case since this is the default) */
	setMode(OPERATION_MODE_CONFIG);

	/* Reset */
	write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
	while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID)
	{
		delay(10);
	} 
	delay(50);
	
	if (calibrate)
	{
		setSensorOffsets(cdata);
	}
	

	/* Set to normal power mode */
	write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
	delay(10);
	write8(BNO055_PAGE_ID_ADDR, 0);

	/* Set the output units */
	
	uint8_t unitsel = (0 << 7) | // Orientation = Android
	                  (0 << 4) | // Temperature = Celsius
	                  (0 << 2) | // Euler = Degrees
	                  (1 << 1) | // Gyro = Rads
	                  (0 << 0);  // Accelerometer = m/s^2
	                write8(BNO055_UNIT_SEL_ADDR, unitsel);
	                

	                /* Configure axis mapping (see section 3.4) */
	                
					write8(BNO055_AXIS_MAP_CONFIG_ADDR, 0x12); //SSK was 0x18);//REMAP_CONFIG_P2); // P0-P7, Default is P1
	                delay(10);
					write8(BNO055_AXIS_MAP_SIGN_ADDR, 0x06); //SSK was 0x04);//REMAP_SIGN_P2); // P0-P7, Default is P1
	                delay(10);
	                
  

	write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
	delay(10);
	/* Set the requested operating mode (see section 3.3) */
	setMode(mode);
	delay(20);

	
	
	return true;
}

/**************************************************************************/

/*!

    @brief  Puts the chip in the specified operating mode

*/
/**************************************************************************/
void BNO055::setMode(adafruit_bno055_opmode_t mode)
{
	_mode = mode;
	write8(BNO055_OPR_MODE_ADDR, _mode);
	delay(30);
}

/**************************************************************************/

/*!

    @brief  Gets the latest system status info

*/

/**************************************************************************/

void BNO055::getSystemStatus(char *system_status, char *self_test_result, char *system_error)
{
	write8(BNO055_PAGE_ID_ADDR, 0);

	/* System Status (see section 4.3.58)
	   ---------------------------------
	   0 = Idle
	   1 = System Error
	   2 = Initializing Peripherals
	   3 = System Iniitalization
	   4 = Executing Self-Test
	   5 = Sensor fusio algorithm running
	   6 = System running without fusion algorithms */

	if (system_status != 0)

		*system_status    = read8(BNO055_SYS_STAT_ADDR);



	/* Self Test Results (see section )

	   --------------------------------

	   1 = test passed, 0 = test failed



	   Bit 0 = Accelerometer self test

	   Bit 1 = Magnetometer self test

	   Bit 2 = Gyroscope self test

	   Bit 3 = MCU self test



	   0x0F = all good! */



	if (self_test_result != 0)

		*self_test_result = read8(BNO055_SELFTEST_RESULT_ADDR);



	/* System Error (see section 4.3.59)

	   ---------------------------------

	   0 = No error

	   1 = Peripheral initialization error

	   2 = System initialization error

	   3 = Self test result failed

	   4 = Register map value out of range

	   5 = Register map address out of range

	   6 = Register map write error

	   7 = BNO low power mode not available for selected operat ion mode

	   8 = Accelerometer power mode not available

	   9 = Fusion algorithm configuration error

	   A = Sensor configuration error */



	if (system_error != 0)

		*system_error     = read8(BNO055_SYS_ERR_ADDR);



	delay(200);

}



/**************************************************************************/

/*!

    @brief  Gets the chip revision numbers

*/

/**************************************************************************/
void BNO055::getRevInfo(adafruit_bno055_rev_info_t* info)
{
	char a, b;
	//memset(info, 0, sizeof(adafruit_bno055_rev_info_t));
	/* Check the accelerometer revision */
	info->accel_rev = read8(BNO055_ACCEL_REV_ID_ADDR);
	/* Check the magnetometer revision */
	info->mag_rev   = read8(BNO055_MAG_REV_ID_ADDR);
	/* Check the gyroscope revision */
	info->gyro_rev  = read8(BNO055_GYRO_REV_ID_ADDR);
	/* Check the SW revision */
	info->bl_rev    = read8(BNO055_BL_REV_ID_ADDR);

	a = read8(BNO055_SW_REV_ID_LSB_ADDR);
	b = read8(BNO055_SW_REV_ID_MSB_ADDR);
	info->sw_rev = (((int)b) << 8) | ((int)a);
}

/**************************************************************************/

/*!

    @brief  Gets current calibration state.  Each value should be a uint8_t

            pointer and it will be set to 0 if not calibrated and 3 if

            fully calibrated.

*/

/**************************************************************************/

void BNO055::getCalibration(char* sys, char* gyro, char* accel, char* mag) {
	uint8_t calData = read8(BNO055_CALIB_STAT_ADDR);
	if (sys != NULL) {
		*sys = (calData >> 6) & 0x03;
	}
	if (gyro != NULL) {
		*gyro = (calData >> 4) & 0x03;
	}
	if (accel != NULL) {
		*accel = (calData >> 2) & 0x03;
	}
	if (mag != NULL) {
		*mag = calData & 0x03;
	}
}

/**************************************************************************/

/*!

    @brief  Gets the temperature in degrees celsius

*/

/**************************************************************************/

char BNO055::getTemp(void)
{
	char temp = (char)(read8(BNO055_TEMP_ADDR));
	return temp;
}

/**********************************************************************************************//**
* Module: BNO055::getVector()
***************************************************************************************************
* @brief	Read a Vector 

* @note 
* Heavily based on the Adadfruit driver provided by KTOWN
* WARNING - this will block until a complete read has happened!!!
*
* @todo 
*
* @param[in,out]	
* @param[in]		
* @returns			
***************************************************************************************************
***************************************************************************************************
* @author
* @li SSK - 11/17/17 - Modified & Commented
**************************************************************************************************/
imu::Vector<3> BNO055::getVector(adafruit_vector_type_t vector_type)

{

	imu::Vector<3> xyz;
	char buffer[6];
	int readError = 0;
	memset(buffer, 0, 6);
	int16_t x, y, z;
	x = y = z = 0;

	int count = 3;
	/* Read vector data (6 bytes) */
	readError = readLen((adafruit_bno055_reg_t)vector_type, buffer, 6);
	count--;
	
	while (readError && count)
	{
		delay(10);		
		readError = readLen((adafruit_bno055_reg_t)vector_type, buffer, 6);		
		count--;
	}
	
	if (!readError)
	{	
		x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
		y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
		z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);
	}
	
	/* Convert the value to an appropriate range (section 3.6.4) */
	/* and assign the value to the Vector type */
	switch (vector_type)

	{

	case VECTOR_MAGNETOMETER:
		/* 1uT = 16 LSB */
		xyz[0] = ((double)x) / 16.0;
		xyz[1] = ((double)y) / 16.0;
		xyz[2] = ((double)z) / 16.0;
		break;

	case VECTOR_GYROSCOPE:
		/* 1dps = 16 LSB */
		xyz[0] = ((double)x) / 16.0;
		xyz[1] = ((double)y) / 16.0;
		xyz[2] = ((double)z) / 16.0;
		break;

	case VECTOR_EULER:
		/* 1 degree = 16 LSB */
		xyz[0] = ((double)x) / 16.0;
		xyz[1] = ((double)y) / 16.0;
		xyz[2] = ((double)z) / 16.0;
		break;

	case VECTOR_ACCELEROMETER:
	case VECTOR_LINEARACCEL:
	case VECTOR_GRAVITY:
		/* 1m/s^2 = 100 LSB */
		xyz[0] = ((double)x) / 100.0;
		xyz[1] = ((double)y) / 100.0;
		xyz[2] = ((double)z) / 100.0;
		break;

	}

	return xyz;
}



/**************************************************************************/

/*!

    @brief  Gets a quaternion reading from the specified source

*/

/**************************************************************************/

imu::Quaternion BNO055::getQuat(int* error)
{
	*error = 0;
	char buffer[8];
	memset(buffer, 0, 8);

	short x, y, z, w;
	x = y = z = w = 0;
	
	/* Read quat data (8 bytes) */
	*error = readLen(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
	w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
	x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
	y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
	z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);



	/* Assign to Quaternion */
	/* See http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf
	   3.6.5.5 Orientation (Quaternion)  */

	const double scale = (1.0 / (1 << 14));
	imu::Quaternion quat(scale * w, scale * x, scale * y, scale * z);
	//qDebug() << "QuatW " << QString::number(w);
	//qDebug() << "QuatX " << QString::number(x);
	//qDebug() << "QuatY " << QString::number(y); 
	//qDebug() << "QuatZ " << QString::number(z); 
	return quat;
}





/**************************************************************************/

/*!

@brief  Reads the sensor's offset registers into a byte array

*/

/**************************************************************************/

bool BNO055::getSensorOffsets(char* calibData)
{
	//if (isFullyCalibrated())
	//{
		adafruit_bno055_opmode_t lastMode = _mode;
		setMode(OPERATION_MODE_CONFIG);
		delay(25);
		readLen(ACCEL_OFFSET_X_LSB_ADDR, calibData, NUM_BNO055_OFFSET_REGISTERS);
		setMode(lastMode);
		return true;
	//}
	//return false;
}

/**************************************************************************/

/*!

@brief  Reads the sensor's offset registers into an offset struct

*/

/**************************************************************************/

bool BNO055::getSensorOffsets(adafruit_bno055_offsets_t &offsets_type)
{
	if (isFullyCalibrated())
	{

		adafruit_bno055_opmode_t lastMode = _mode;

		setMode(OPERATION_MODE_CONFIG);

		delay(25);



		offsets_type.accel_offset_x = (read8(ACCEL_OFFSET_X_MSB_ADDR) << 8) | (read8(ACCEL_OFFSET_X_LSB_ADDR));

		offsets_type.accel_offset_y = (read8(ACCEL_OFFSET_Y_MSB_ADDR) << 8) | (read8(ACCEL_OFFSET_Y_LSB_ADDR));

		offsets_type.accel_offset_z = (read8(ACCEL_OFFSET_Z_MSB_ADDR) << 8) | (read8(ACCEL_OFFSET_Z_LSB_ADDR));



		offsets_type.gyro_offset_x = (read8(GYRO_OFFSET_X_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_X_LSB_ADDR));

		offsets_type.gyro_offset_y = (read8(GYRO_OFFSET_Y_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_Y_LSB_ADDR));

		offsets_type.gyro_offset_z = (read8(GYRO_OFFSET_Z_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_Z_LSB_ADDR));



		offsets_type.mag_offset_x = (read8(MAG_OFFSET_X_MSB_ADDR) << 8) | (read8(MAG_OFFSET_X_LSB_ADDR));

		offsets_type.mag_offset_y = (read8(MAG_OFFSET_Y_MSB_ADDR) << 8) | (read8(MAG_OFFSET_Y_LSB_ADDR));

		offsets_type.mag_offset_z = (read8(MAG_OFFSET_Z_MSB_ADDR) << 8) | (read8(MAG_OFFSET_Z_LSB_ADDR));



		offsets_type.accel_radius = (read8(ACCEL_RADIUS_MSB_ADDR) << 8) | (read8(ACCEL_RADIUS_LSB_ADDR));

		offsets_type.mag_radius = (read8(MAG_RADIUS_MSB_ADDR) << 8) | (read8(MAG_RADIUS_LSB_ADDR));



		setMode(lastMode);

		return true;

	}

	return false;

}





/**************************************************************************/

/*!

@brief  Writes an array of calibration values to the sensor's offset registers

*/

/**************************************************************************/

void BNO055::setSensorOffsets(const char* calibData)
{

	adafruit_bno055_opmode_t lastMode = _mode;

	setMode(OPERATION_MODE_CONFIG);

	delay(25);



	/* A writeLen() would make this much cleaner */

	write8(ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);

	write8(ACCEL_OFFSET_X_MSB_ADDR, calibData[1]);

	write8(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]);

	write8(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]);

	write8(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]);

	write8(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]);



	write8(GYRO_OFFSET_X_LSB_ADDR, calibData[6]);

	write8(GYRO_OFFSET_X_MSB_ADDR, calibData[7]);

	write8(GYRO_OFFSET_Y_LSB_ADDR, calibData[8]);

	write8(GYRO_OFFSET_Y_MSB_ADDR, calibData[9]);

	write8(GYRO_OFFSET_Z_LSB_ADDR, calibData[10]);

	write8(GYRO_OFFSET_Z_MSB_ADDR, calibData[11]);



	write8(MAG_OFFSET_X_LSB_ADDR, calibData[12]);

	write8(MAG_OFFSET_X_MSB_ADDR, calibData[13]);

	write8(MAG_OFFSET_Y_LSB_ADDR, calibData[14]);

	write8(MAG_OFFSET_Y_MSB_ADDR, calibData[15]);

	write8(MAG_OFFSET_Z_LSB_ADDR, calibData[16]);

	write8(MAG_OFFSET_Z_MSB_ADDR, calibData[17]);



	write8(ACCEL_RADIUS_LSB_ADDR, calibData[18]);

	write8(ACCEL_RADIUS_MSB_ADDR, calibData[19]);



	write8(MAG_RADIUS_LSB_ADDR, calibData[20]);

	write8(MAG_RADIUS_MSB_ADDR, calibData[21]);



	setMode(lastMode);

}



/**************************************************************************/

/*!

@brief  Writes to the sensor's offset registers from an offset struct

*/

/**************************************************************************/
void BNO055::setSensorOffsets(const adafruit_bno055_offsets_t &offsets_type)
{

	adafruit_bno055_opmode_t lastMode = _mode;

	setMode(OPERATION_MODE_CONFIG);

	delay(25);



	write8(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type.accel_offset_x) & 0x0FF);

	write8(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type.accel_offset_x >> 8) & 0x0FF);

	write8(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type.accel_offset_y) & 0x0FF);

	write8(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type.accel_offset_y >> 8) & 0x0FF);

	write8(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type.accel_offset_z) & 0x0FF);

	write8(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type.accel_offset_z >> 8) & 0x0FF);



	write8(GYRO_OFFSET_X_LSB_ADDR, (offsets_type.gyro_offset_x) & 0x0FF);

	write8(GYRO_OFFSET_X_MSB_ADDR, (offsets_type.gyro_offset_x >> 8) & 0x0FF);

	write8(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type.gyro_offset_y) & 0x0FF);

	write8(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type.gyro_offset_y >> 8) & 0x0FF);

	write8(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type.gyro_offset_z) & 0x0FF);

	write8(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type.gyro_offset_z >> 8) & 0x0FF);



	write8(MAG_OFFSET_X_LSB_ADDR, (offsets_type.mag_offset_x) & 0x0FF);

	write8(MAG_OFFSET_X_MSB_ADDR, (offsets_type.mag_offset_x >> 8) & 0x0FF);

	write8(MAG_OFFSET_Y_LSB_ADDR, (offsets_type.mag_offset_y) & 0x0FF);

	write8(MAG_OFFSET_Y_MSB_ADDR, (offsets_type.mag_offset_y >> 8) & 0x0FF);

	write8(MAG_OFFSET_Z_LSB_ADDR, (offsets_type.mag_offset_z) & 0x0FF);

	write8(MAG_OFFSET_Z_MSB_ADDR, (offsets_type.mag_offset_z >> 8) & 0x0FF);



	write8(ACCEL_RADIUS_LSB_ADDR, (offsets_type.accel_radius) & 0x0FF);

	write8(ACCEL_RADIUS_MSB_ADDR, (offsets_type.accel_radius >> 8) & 0x0FF);



	write8(MAG_RADIUS_LSB_ADDR, (offsets_type.mag_radius) & 0x0FF);

	write8(MAG_RADIUS_MSB_ADDR, (offsets_type.mag_radius >> 8) & 0x0FF);



	setMode(lastMode);

}



bool BNO055::isFullyCalibrated(void)
{
	char system, gyro, accel, mag;

	getCalibration(&system, &gyro, &accel, &mag);

	if (system < 3 || gyro < 3 || accel < 3 || mag < 3)
		return false;
	return true;
}





/***************************************************************************

 PRIVATE FUNCTIONS

 ***************************************************************************/



/**************************************************************************/

/*!

    @brief  Writes an 8 bit value over I2C

*/

/**************************************************************************/

bool BNO055::write8(adafruit_bno055_reg_t reg, char value)
{
	char uartbuff[5];
	uartbuff[0] = 0xAA;
	uartbuff[1] = 0x00;
	uartbuff[2] = (char)reg;
	uartbuff[3] = 1;
	uartbuff[4] = value;
	
	char resp[2];
	//Wire.beginTransmission(_address);
	//bcm2835_i2c_setSlaveAddress(_address);
	//Wire.send(reg);
	//bcm2835_i2c_write((char*)&reg, 1);
	tcflush(sfd, TCIOFLUSH);
	int count = write(sfd, uartbuff, 5);
	tcdrain(sfd);
	//bcm2835_i2c_write((char*)&value, 1);
	//Wire.endTransmission();
	delay(1);
	int bytes;
	ioctl(sfd, FIONREAD, &bytes);
	if (bytes == 2) {
		count = read(sfd, resp, 2);
	}
	if (resp[0] == 0xEE && resp[1] == 0x01)
		return true;
	
	/* ToDo: Check for error! */
	return false;
}



/**************************************************************************/

/*!

    @brief  Reads an 8 bit value over I2C

*/

/**************************************************************************/

char BNO055::read8(adafruit_bno055_reg_t reg)

{
	char value = 0;
	char uartbuff[4];
	uartbuff[0] = 0xAA;
	uartbuff[1] = 0x01;
	uartbuff[2] = (char)reg;
	uartbuff[3] = 1;
	
	char resp[3];
	//Wire.beginTransmission(_address);
	//bcm2835_i2c_setSlaveAddress(_address);
	//Wire.send(reg);
	//bcm2835_i2c_write((char*)&reg, 1);
	tcflush(sfd, TCIOFLUSH);
	int count = write(sfd, uartbuff, 4);
	tcdrain(sfd);
	//bcm2835_i2c_write((char*)&value, 1);
	//Wire.endTransmission();
	delay(1);
	int bytes;
	ioctl(sfd, FIONREAD, &bytes);
	if (bytes == 3) {
		count = read(sfd, resp, bytes);
	}
	if (resp[0] == 0xEE && resp[1] == 0x02)
		return 0;
	if (resp[0] == 0xBB && resp[1] == 0x01) value = resp[2];
	/* ToDo: Check for error! */
	
	
	return value;
}



/**********************************************************************************************//**
* Module: BNO055::readLen()
***************************************************************************************************
* @brief	Read a Number of Bytes off of the UART

* @note		Raspberry pi doesn't support clock stretching, so we have to use UART for now...
*
* @todo 
*
* @param[in,out]	
* @param[in]		
* @returns	Error
*			0 = no error
*			1 = write request failed
*			2 = no bytes in receive buffer
*			16 + n = got an EE for the response - error from BNO055, response code is error - 16;
*			4 = unknown response initial byte
*			5 = response longer than requested length
*			6 = response shorter than requested length
***************************************************************************************************
***************************************************************************************************
* @author
* @li SSK - 11/02/17 - Created & Commented
**************************************************************************************************/
int BNO055::readLen(adafruit_bno055_reg_t reg, char* buffer, char len)
{
	int error = 0;
	
	char uartbuff[4];
	uartbuff[0] = 0xAA;
	uartbuff[1] = 0x01;
	uartbuff[2] = (char)reg;
	uartbuff[3] = len;
	
	char resp[2];
	//Wire.beginTransmission(_address);
	//bcm2835_i2c_setSlaveAddress(_address);
	//Wire.send(reg);
	//bcm2835_i2c_write((char*)&reg, 1);
	tcflush(sfd, TCIOFLUSH);
	int count = write(sfd, uartbuff, 4);
	if (count != 4) error = 1;
	tcdrain(sfd);
	//bcm2835_i2c_write((char*)&value, 1);
	//Wire.endTransmission();
	delay(1);
	if (!error)
	{
		int bytes;
		ioctl(sfd, FIONREAD, &bytes);
		if (bytes >= 2) 
		{
			count = read(sfd, resp, 2);
			if (resp[0] == 0xEE)// && resp[1] == 0x02) 
			{
				error = 16 + resp[1];
			}
			if (!error)
			{		
				if (resp[0] == 0xBB)
				{
					if (resp[1] <= len)
					{
						delay(1);
						ioctl(sfd, FIONREAD, &bytes);
						if (bytes >= resp[1])
						{
							read(sfd, buffer, resp[1]);	
						}
						else
						{
							error = 6;
						}
					}
					else
					{
						error = 5;
					}
				}
				else
				{
					error = 4;
				}
			}
		}
		else
		{
			error = 2;
		}
	}
	/* ToDo: Check for error! */
	

	return error;

}

uint BNO055::readRegister(uint reg)
{
	return((uint)read8((adafruit_bno055_reg_t)reg));
}


void BNO055::writeRegister(uint reg, uint val)
{
	write8((adafruit_bno055_reg_t)reg, (char)val);
}
