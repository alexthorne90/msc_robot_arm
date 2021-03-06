/**
 * @file ldc1614.cpp
 * @author Alex Thorne
 * @version 1.0
 */

#include "ldc1614.h"

//#define LDC1614_DEBUG

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

#ifdef LDC1614_DEBUG
    Serial.print("Wrote ");
    Serial.print(reg);
    Serial.print("to reg ");
    Serial.println(value);
#endif

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

#ifdef LDC1614_DEBUG
    Serial.print("Read reg ");
    Serial.print(reg);
    Serial.print("    and got: ");
    Serial.println(value);
#endif

    return value;
}

bool Ldc1614::TestConnection(void)
{
    return ReadReg(DEVICE_ID_REG) == LDC1614_DEVICE_ID;
}
