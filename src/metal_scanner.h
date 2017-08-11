/**
 * @file metal_scanner_lib.h
 * @author Alex Thorne
 * @version 1.0
 */

#ifndef METAL_SCANNER_LIB_H
#define METAL_SCANNER_LIB_H

#include <Arduino.h>
#include "arm_controller.h"
#include "metal_detector_lib.h"

class MetalScanner : public ArmController {

    public:

        //Constructor
        MetalScanner(void);

        //Public interface
        void HardwareSetup(void);
        void SetReferenceInductance(void);
        uint8_t SetScanOrigin(float x, float y, float z);
        uint8_t SetDesiredInductanceDelta(float inductance);
        uint8_t SetInductanceDeltaTolerance(float tolerance);
        void Update(uint16_t time_since_last_update_ms);

    private:

        //Private variables
        MetalDetector metal_detector;
        float reference_inductance;
        float origin_x;
        float origin_y;
        float origin_z;
        float desired_delta_uH;
        float uH_tolerance;

        //Default settings
        const float DEFAULT_DELTA_uH = 0.25;
        const float DEFAULT_uH_TOLERANCE = 0.1;

        //Private helper functions
};

#endif


