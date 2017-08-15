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
        uint8_t SetDepthScanIncrementMM(float depth_increment);
        uint8_t SetTravelIncrementMM(float travel_increment);
        uint8_t SetHeightCorrectionMM(float height_correction);
        void Update(uint16_t time_since_last_update_ms);
        bool isScanComplete(void);
        void ResetScan(void);
        float GetOutOfBoundsPercentage(void);

    private:

        typedef enum scan_state {
            S0_IDLE,
            S1_MOVE_ABOVE_ORIGIN,
            S2_FIND_HEIGHT,
            S3_HORIZONTAL_MOVE_FWD,
            S4_DEPTH_MOVE,
            S5_HORIZONTAL_MOVE_BACK,
            S6_CYCLE_END,
            S7_SCAN_FINISHED
        } scan_state;

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
        float travel_increment_mm;
        float height_correction_mm;
        scan_state current_state;
        scan_state next_state;
        bool transitioned_state;
        uint16_t current_cycle_count;
        float current_inductance;
        float inductance_delta;
        float current_x;
        float current_y;
        float current_z;
        uint32_t move_counter;
        uint32_t out_of_bounds_counter;

        //Default settings
        const float DEFAULT_DELTA_uH = 0.25;
        const float DEFAULT_uH_TOLERANCE = 0.1;
        const float DEFAULT_HORIZONTAL_DESIRED_TRAVEL_MM = 50;
        const float DEFAULT_DEPTH_DESIRED_TRAVEL_MM = 25;
        const float DEFAULT_DEPTH_SCAN_INCREMENT_MM = 5;
        const float DEFAULT_TRAVEL_INCREMENT_MM = 1.0;
        const float DEFAULT_HEIGHT_CORRECTION_MM = 1.0;
        const float DISTANCE_ABOVE_ORIGIN_START_MM = 60.0;

        //Private helper functions
        float GetCorrectedHeight(float current_z, float inductance);
        scan_state S0_Run(void);
        scan_state S1_Run(void);
        scan_state S2_Run(void);
        scan_state S3_Run(void);
        scan_state S4_Run(void);
        scan_state S5_Run(void);
        scan_state S6_Run(void);
        scan_state S7_Run(void);
};

#endif


