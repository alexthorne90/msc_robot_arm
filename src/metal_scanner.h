/**
 * @file metal_scanner.h
 * @author Alex Thorne
 * @version 1.0
 */

#ifndef METAL_SCANNER_H
#define METAL_SCANNER_H

#include <Arduino.h>
#include "arm_controller.h"
#include "metal_detector.h"

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
        virtual void Update(uint16_t time_since_last_update_ms) = 0;
        virtual bool isScanComplete(void) = 0;
        virtual void ResetScan(void) = 0;
        virtual float GetOutOfBoundsPercentage(void) = 0;

    protected:

        //Private variables
        MetalDetector metal_detector;
        float reference_inductance;
        float origin_x;
        float origin_y;
        float origin_z;
        float desired_delta_uH;
        float uH_tolerance;

        //Default settings
        const float DEFAULT_DELTA_uH = 1.0;
        const float DEFAULT_uH_TOLERANCE = 0.4;
        const float DISTANCE_ABOVE_ORIGIN_START_MM = 60.0;

        //Boundaries
        const float X_MIN_BOUNDARY = -130.0;
        const float X_MAX_BOUNDARY = 130.0;
        const float Y_MIN_BOUNDARY = 80.0;
        const float Y_MAX_BOUNDARY = 230.0;
        const float Z_MIN_BOUNDARY = 5.0;
        const float Z_MAX_BOUNDARY = 100.0;
};

#endif


