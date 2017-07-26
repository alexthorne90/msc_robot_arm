/**
 * @file ldc1614_lib.cpp
 * @author Alex Thorne
 * @version 1.0
 */

#include "ldc1614_lib.h"

#define LDC1614_DEBUG

Ldc1614::Ldc1614()
{
	//initialize variables
}

void Ldc1614::AttachComms(void)
{
    Wire.begin();
}

uint8_t Ldc1614::WriteReg(uint8_t reg, uint16_t value)
{
    uint8_t msb = 0;
    uint8_t lsb = 0;
    msb = value >> 8;
    lsb = value & 0x00FF;
    Wire.beginTransmission(LDC_BUS_ADDR);
    Wire.write(reg);
    Wire.write(msb);
    Wire.write(lsb);
    Wire.endTransmission();
    return 0;
}

uint16_t Ldc1614::ReadReg(uint8_t reg)
{
    uint8_t msb = 0;
    uint8_t lsb = 0;
    uint16_t value = 0;
    Wire.beginTransmission(LDC_BUS_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(LDC_BUS_ADDR, (uint8_t)2);
    while (Wire.available())
    {
        msb = Wire.read();
        lsb = Wire.read();
    }
    value = msb;
    value <<= 8;
    value += lsb;
    return value;
}
