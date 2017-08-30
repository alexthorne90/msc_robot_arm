/**
 * @file shape_mapping_scanner.h
 * @author Alex Thorne
 * @version 1.0
 */

#ifndef SHAPE_MAPPING_SCANNER_H
#define SHAPE_MAPPING_SCANNER_H

#include "metal_scanner.h"

class ShapeMappingScanner : public MetalScanner {

    public:

        //Constructor
        ShapeMappingScanner(void);

        //Public interface
        void Update(uint16_t time_since_last_update_ms);
        bool isScanComplete(void);
        void ResetScan(void);
        float GetOutOfBoundsPercentage(void);

    private:

        typedef enum scan_state {
            S0_IDLE,
            S1_MOVE_ABOVE_ORIGIN,
            S2_FIND_HEIGHT,
        } scan_state;

        //Private variables
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
        const float DEFAULT_DEPTH_SCAN_INCREMENT_MM = 0;
        const float DEFAULT_TRAVEL_INCREMENT_MM = 0.5;
        const float DEFAULT_HEIGHT_CORRECTION_MM = 1.0;

        //Private helper functions
        float GetCorrectedHeight(float current_z, float inductance);
        scan_state S0_Run(void);
        scan_state S1_Run(void);
        scan_state S2_Run(void);
};

#endif

