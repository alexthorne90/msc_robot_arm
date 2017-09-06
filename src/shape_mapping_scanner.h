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
            S3_HORIZONTAL_MOVE_FWD,
            S4_HORIZ_FWD_START_TEST,
            S5_HORIZ_FWD_EDGE_TEST,
            S6_HORIZ_FWD_EDGE_RECOVERY,
            S7_RESTABILIZE_SCAN,
            S8_HORIZ_FWD_EDGE_CONFIRMED,
            S9_TEST_SMALL_LOWER,
            S10_INC_SMALL_LOWER_TEST,
            S12_SCAN_COMPLETE
        } scan_state;

        //Private variables
        float depth_scan_increment_mm;
        float travel_increment_mm;
        float height_correction_mm;
        scan_state current_state;
        scan_state next_state;
        bool transitioned_state;
        uint16_t current_cycle_count;
        float previous_inductance;
        float current_inductance;
        float inductance_delta;
        float current_x;
        float current_y;
        float current_z;
        uint32_t move_counter;
        uint32_t out_of_bounds_counter;
        uint32_t down_counter;
        uint32_t back_in_range_count;
        float edge_x;
        float edge_z;

        //Default settings
        const float DEFAULT_DEPTH_SCAN_INCREMENT_MM = 0;
        const float DEFAULT_TRAVEL_INCREMENT_MM = 0.5;
        const float DEFAULT_HEIGHT_CORRECTION_MM = 1.0;
        const uint32_t EDGE_DETECTION_COUNT = 2;

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
        scan_state S8_Run(void);
        scan_state S9_Run(void);
        scan_state S10_Run(void);
        scan_state S12_Run(void);
        bool hasDetectedEdge();
};

#endif

