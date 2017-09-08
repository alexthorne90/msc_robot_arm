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

        //Public types
        typedef struct edge_point {
            float x_coordinate;
            float y_coordinate;
        } edge_point;

    private:

        typedef enum scan_state {
            S0_IDLE,
            S1_MOVE_ABOVE_ORIGIN,
            S2_FIND_HEIGHT,
            S3_SCANNING,
            S4_BEGIN_EDGE_TEST,
            S5_EDGE_TEST_LOWERING,
            S6_EDGE_CONFIRMED,
            S7_EDGE_TEST_POSITION_INC,
            S8_EDGE_DISPROVED,
            S9_RESET_SCAN,
            S10_READY_NEXT_SCAN,
            S11_SCAN_COMPLETE
        } scan_state;

        typedef enum scan_direction {
            X_FORWARD,
            X_BACKWARD,
            Y_FORWARD,
            Y_BACKWARD
        } scan_direction;

        //Scan settings
        const float DEFAULT_DEPTH_SCAN_INCREMENT_MM = 0;
        const float TRAVEL_INCREMENT_MM = 0.5;
        const float HEIGHT_CORRECTION_MM = 1.0;
        const float EDGE_TEST_HEIGHT_DROP_MM = 2.5;
        const float EDGE_TEST_HORIZ_INC_MM = 3.5;
        const float EDGE_TEST_HORIZ_RANGE_MM = 14.0;
        const float HEIGHT_SAFETY_DIST_MM = 20.0;
        const uint32_t EDGE_DETECTION_COUNT = 2;
        static const uint8_t NUM_EDGES = 12;

        //Private variables
        float depth_scan_increment_mm;
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
        uint32_t down_counter;
        float edge_x;
        float edge_y;
        float edge_z;
        scan_direction current_scan_dir;
        uint8_t edge_counter;
        edge_point edge_map[NUM_EDGES];

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
        scan_state S11_Run(void);
        bool hasDetectedEdge(void);
        bool isInductanceTooLow(void);
        bool hasPassedEndOfEdgeTestRange(void);
        void SetArmScanning(void);
        void SetArmEdgeTestPosition(void);
        void SetArmResettingScan(void);
        void SetArmWithNextScanOrigin(void);

};

#endif

