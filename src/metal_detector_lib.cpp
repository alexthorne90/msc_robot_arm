/**
 * @file metal_detector_lib.cpp
 * @author Alex Thorne
 * @version 1.0
 */

#include "metal_detector_lib.h"

//#define METAL_DETECTOR_DEBUG

MetalDetector::MetalDetector() : Ldc1614()
{
	//initialize variables
    channel0_raw_value = 0;
    channel0_freq_hz = 0;
    channel0_L_uH = 0.0;
}

void MetalDetector::ConnectToSensor(void)
{
    AttachComms();
}

uint8_t MetalDetector::ConfigureSensor(void)
{
    WriteReg(RCOUNT_CH0, 0xFFFF);
    WriteReg(SETTLECOUNT_CH0, 0x002D);
    WriteReg(CLK_DIVIDERS_CH0, 0x1002);
    WriteReg(ERROR_CONFIG, 0x0000);
    WriteReg(MUX_CONFIG_REG, 0x420C);
    WriteReg(DRIVE_CURRENT_CH0, 0x0680);
    WriteReg(CONFIG_REG, 0x0001);
    return 0;
}

uint8_t MetalDetector::Update(void)
{
    uint16_t ch0_msb;
    uint16_t ch0_lsb;
    float ch0_conv;
    ch0_msb = ReadReg(DATA_MSB_CH0);
    ch0_lsb = ReadReg(DATA_LSB_CH0);
    channel0_raw_value = ch0_msb;
    channel0_raw_value <<= 16;
    channel0_raw_value |= ch0_lsb;

#ifdef METAL_DETECTOR_DEBUG
    Serial.print("Raw ch0 value  ");
    Serial.println(channel0_raw_value);
#endif

    ch0_conv = (float)channel0_raw_value * 0.08076;
    channel0_freq_hz = (uint32_t) ch0_conv;

    channel0_L_uH = 1000000.0 * (1.0 / 0.000000001) * pow(
            1.0 / (2.0 * PI * (float)channel0_freq_hz), 2);

    return 0;
}

uint32_t MetalDetector::GetCh0FreqHz(void)
{
    return channel0_freq_hz;
}

float MetalDetector::GetCh0InductanceuH(void)
{
    return channel0_L_uH;
}
