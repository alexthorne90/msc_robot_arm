/**
 * @file metal_detector.h
 * @author Alex Thorne
 * @version 1.0
 */

#ifndef METAL_DETECTOR_H
#define METAL_DETECTOR_H

#include <Arduino.h>
#include "ldc1614.h"

class MetalDetector : public Ldc1614 {

    public:

        MetalDetector();

        void ConnectToSensor(void);
        uint8_t ConfigureSensor(void);
        uint8_t Update(void);
        uint32_t GetCh0FreqHz(void);
        float GetCh0InductanceuH(void);

    private:

        //Variables
        uint32_t channel0_raw_value;
        uint32_t channel0_freq_hz;
        float channel0_L_uH;
};

#endif


