/**
 * @file ldc1614_lib.h
 * @author Alex Thorne
 * @version 1.0
 */

#ifndef LDC1614_LIB_H
#define LDC1614_LIB_H

#include <Arduino.h>
#include <Wire.h>

class Ldc1614 {

    public:

        Ldc1614();

        void AttachComms();
        uint8_t WriteReg(uint8_t reg, uint16_t value);
        uint16_t ReadReg(uint8_t reg);

    private:

        //LDC1614 bus address
        const uint8_t LDC_BUS_ADDR          = 0x2A;

        //LDC1614 IDs
        const uint16_t LDC1614_MANUFACT_ID  = 0x5449;
        const uint16_t LDC1614_DEVICE_ID    = 0x3005;

        //LDC1614 register addresses
        //  Data
        const uint8_t DATA_MSB_CH0      = 0x00;
        const uint8_t DATA_LSB_CH0      = 0x01;
        const uint8_t DATA_MSB_CH1      = 0x02;
        const uint8_t DATA_LSB_CH1      = 0x03;
        const uint8_t DATA_MSB_CH2      = 0x04;
        const uint8_t DATA_LSB_CH2      = 0x05;
        const uint8_t DATA_MSB_CH3      = 0x06;
        const uint8_t DATA_LSB_CH3      = 0x07;
        //  Reference count
        const uint8_t RCOUNT_CH0        = 0x08;
        const uint8_t RCOUNT_CH1        = 0x09;
        const uint8_t RCOUNT_CH2        = 0x0A;
        const uint8_t RCOUNT_CH3        = 0x0B;
        //  Offset value
        const uint8_t OFFSET_CH0        = 0x0C;
        const uint8_t OFFSET_CH1        = 0x0D;
        const uint8_t OFFSET_CH2        = 0x0E;
        const uint8_t OFFSET_CH3        = 0x0F;
        //  Settling referece count
        const uint8_t SETTLECOUNT_CH0   = 0x10;
        const uint8_t SETTLECOUNT_CH1   = 0x11;
        const uint8_t SETTLECOUNT_CH2   = 0x12;
        const uint8_t SETTLECOUNT_CH3   = 0x13;
        //  Reference and sensor divider settings
        const uint8_t CLK_DIVIDERS_CH0  = 0x14;
        const uint8_t CLK_DIVIDERS_CH1  = 0x15;
        const uint8_t CLK_DIVIDERS_CH2  = 0x16;
        const uint8_t CLK_DIVIDERS_CH3  = 0x17;
        //  Device status report
        const uint8_t STATUS_REG        = 0x18;
        //  Error reporting config
        const uint8_t ERROR_CONFIG      = 0x19;
        //  Conversion config
        const uint8_t CONFIG_REG        = 0x1A;
        //  Channel multiplexing config
        const uint8_t MUX_CONFIG_REG    = 0x1B;
        //  Reset device
        const uint8_t RESET_DEV_REG     = 0x1C;
        //  Sensor current drive configuration
        const uint8_t DRIVE_CURRENT_CH0 = 0x1E;
        const uint8_t DRIVE_CURRENT_CH1 = 0x1F;
        const uint8_t DRIVE_CURRENT_CH2 = 0x20;
        const uint8_t DRIVE_CURRENT_CH3 = 0x21;
        //  Sensor current drive configuration
        const uint8_t MANUFACT_ID_REG   = 0x22;
        const uint8_t DEVICE_ID_REG     = 0x23;
};

#endif

