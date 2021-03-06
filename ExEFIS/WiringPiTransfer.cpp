/*  Code for WiringPi implementations of ByteTransfer class
 *
 *  Copyright 2018 Simon D. Levy
 *  
 *  Library may be used freely and without limit with attribution.
 */

#include "WiringPiTransfer.h"

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>



void WiringPiI2C::begin(void)
{	
    _fd = wiringPiI2CSetup (_address);
}

void WiringPiI2C::writeRegister(uint8_t subAddress, uint8_t data)
{
	wiringPiI2CWriteReg8(_fd, subAddress, data);
}

void WiringPiI2C::readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
    for (uint8_t i=0; i<count; ++i) {
        dest[i] = wiringPiI2CReadReg8(_fd, subAddress+i);
    }
}


void WiringPiSPI::begin(void)
{
    wiringPiSPISetup(_bus, _speed);
}

void WiringPiSPI::writeRegister(uint8_t subAddress, uint8_t data)
{
    uint8_t buff2[2];
    buff2[0] = subAddress;
    buff2[1] = data;
    wiringPiSPIDataRW(_bus, &buff2[0], 2);
}

void WiringPiSPI::readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
     uint8_t buff2[2];
     for (uint8_t i=0; i<count; ++i) {
         buff2[0] = (subAddress+i) | 0x80;
         buff2[1] = 0;
         wiringPiSPIDataRW(_bus, &buff2[0], 2);
         dest[i] = buff2[1];
     }
}
