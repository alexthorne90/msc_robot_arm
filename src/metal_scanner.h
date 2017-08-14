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
        uint8_t SetDesiredHorizontalTravelMM(float horizontal_travel);
        uint8_t SetDesiredDepthTravelMM(float depth_travel);
        uint8_t SetDepthScanIncrement(float depth_increment);
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
        float horizontal_desired_travel_mm;
        float depth_desired_travel_mm;
        float depth_scan_increment_mm;

        //Default settings
        const float DEFAULT_DELTA_uH = 0.25;
        const float DEFAULT_uH_TOLERANCE = 0.1;
        const float DEFAULT_HORIZONTAL_DESIRED_TRAVEL_MM = 50;
        const float DEFAULT_DEPTH_DESIRED_TRAVEL_MM = 25;
        const float DEFAULT_DEPTH_SCAN_INCREMENT_MM = 5;

        //Private helper functions
};

#endif


