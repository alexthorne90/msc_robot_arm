/**
 * @file metal_detector_lib.cpp
 * @author Alex Thorne
 * @version 1.0
 */

#include "metal_detector_lib.h"

//#define METAL_DETECTOR_DEBUG

MetalDetector::MetalDetector()
{
	//initialize variables
    channel0_raw_value = 0;
    channel0_freq_hz = 0;
    channel0_L_uH = 0.0;
}

void MetalDetector::ConnectToSensor(void)
{
    ldc_sensor.AttachComms();
}

uint8_t MetalDetector::ConfigureSensor(void)
{
    ldc_sensor.WriteReg(ldc_sensor.RCOUNT_CH0, 0xFFFF);
    ldc_sensor.WriteReg(ldc_sensor.SETTLECOUNT_CH0, 0x002D);
    ldc_sensor.WriteReg(ldc_sensor.CLK_DIVIDERS_CH0, 0x1002);
    ldc_sensor.WriteReg(ldc_sensor.ERROR_CONFIG, 0x0000);
    ldc_sensor.WriteReg(ldc_sensor.MUX_CONFIG_REG, 0x420C);
    ldc_sensor.WriteReg(ldc_sensor.DRIVE_CURRENT_CH0, 0x0680);
    ldc_sensor.WriteReg(ldc_sensor.CONFIG_REG, 0x0001);
    return 0;
}

uint8_t MetalDetector::Update(void)
{
    uint16_t ch0_msb;
    uint16_t ch0_lsb;
    float ch0_conv;
    ch0_msb = ldc_sensor.ReadReg(ldc_sensor.DATA_MSB_CH0);
    ch0_lsb = ldc_sensor.ReadReg(ldc_sensor.DATA_LSB_CH0);
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
