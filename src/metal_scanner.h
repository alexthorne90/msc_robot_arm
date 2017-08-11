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
        void Update(uint16_t time_since_last_update_ms);

    private:

        //Private variables
        MetalDetector metal_detector;
        float reference_inductance;

        //Private helper functions
};

#endif


