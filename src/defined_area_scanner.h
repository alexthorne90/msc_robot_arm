/**
 * @file defined_area_scanner.h
 * @author Alex Thorne
 * @version 1.0
 */

#ifndef DEFINED_AREA_SCANNER_H
#define DEFINED_AREA_SCANNER_H

#include "metal_scanner.h"

class DefinedAreaScanner : public MetalScanner {

    public:

        //Constructor
        DefinedAreaScanner(void);

        //Public interface
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
        const float DEFAULT_HORIZONTAL_DESIRED_TRAVEL_MM = 0;
        const float DEFAULT_DEPTH_DESIRED_TRAVEL_MM = 0;
        const float DEFAULT_DEPTH_SCAN_INCREMENT_MM = 0;
        const float DEFAULT_TRAVEL_INCREMENT_MM = 0.5;
        const float DEFAULT_HEIGHT_CORRECTION_MM = 1.0;

        //Boundaries
        const float MAX_TRAVEL_INCREMENT_MM = 14.0;
        const float MAX_HEIGHT_ADJUSTMENT_MM = 5.0;

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
        bool isScanAreaValid(void);
};

#endif
