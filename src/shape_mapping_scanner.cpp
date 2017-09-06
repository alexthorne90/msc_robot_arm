/**
 * @file shape_mapping_scanner.cpp
 * @author Alex Thorne
 * @version 1.0
 */

#include "shape_mapping_scanner.h"

//#define SHAPE_MAPPING_SCANNER_DEBUG

ShapeMappingScanner::ShapeMappingScanner(void) : MetalScanner()
{
	//initialize
    depth_scan_increment_mm = DEFAULT_DEPTH_SCAN_INCREMENT_MM;
    travel_increment_mm = DEFAULT_TRAVEL_INCREMENT_MM;
    height_correction_mm = DEFAULT_HEIGHT_CORRECTION_MM;
    current_state = S0_IDLE;
    next_state = S0_IDLE;
    transitioned_state = false;
}

void ShapeMappingScanner::Update(uint16_t time_since_last_update_ms)
{
    metal_detector.Update();
    current_inductance = metal_detector.GetCh0InductanceuH();
    //Serial.print("Current inductance = ");
    //Serial.println(current_inductance, 6);
    inductance_delta = reference_inductance - current_inductance;
    current_x = GetCurrentX();
    current_y = GetCurrentY();
    current_z = GetCurrentZ();

    switch (current_state)
    {
        case S0_IDLE:
            next_state = S0_Run();
            break;
        case S1_MOVE_ABOVE_ORIGIN:
            next_state = S1_Run();
            break;
        case S2_FIND_HEIGHT:
            next_state = S2_Run();
            break;
        case S3_HORIZONTAL_MOVE_FWD:
            next_state = S3_Run();
            break;
        case S4_HORIZ_FWD_START_TEST:
            next_state = S4_Run();
            break;
        case S5_HORIZ_FWD_EDGE_TEST:
            next_state = S5_Run();
            break;
        case S6_HORIZ_FWD_EDGE_RECOVERY:
            next_state = S6_Run();
            break;
        case S7_RESTABILIZE_SCAN:
            next_state = S7_Run();
            break;
        case S8_HORIZ_FWD_EDGE_CONFIRMED:
            next_state = S8_Run();
            break;
        case S9_TEST_SMALL_LOWER:
            next_state = S9_Run();
            break;
        case S10_INC_SMALL_LOWER_TEST:
            next_state = S10_Run();
            break;
        case S12_SCAN_COMPLETE:
            next_state = S12_Run();
            break;
        default:
            break;
    }
    if (current_state != next_state)
    {
#ifdef SHAPE_MAPPING_SCANNER_DEBUG
        Serial.print("Transitioning to state ");
        Serial.println(next_state);
#endif
        transitioned_state = true;
    }
    else
    {
        transitioned_state = false;
    }
    current_state = next_state;

    ArmController::Update(time_since_last_update_ms);
    previous_inductance = current_inductance;
}

bool ShapeMappingScanner::isScanComplete(void)
{
    return (current_state == S12_SCAN_COMPLETE);
}

void ShapeMappingScanner::ResetScan(void)
{
    current_state = S0_IDLE;
    next_state = S0_IDLE;
}

float ShapeMappingScanner::GetOutOfBoundsPercentage(void)
{
    if (move_counter <= 0)
    {
        return 0;
    }
    return ((float)out_of_bounds_counter / (float)move_counter * 100.0);
}

// Private functions ***********************************************************
float ShapeMappingScanner::GetCorrectedHeight(float current_z, float inductance)
{
    float corrected_z = current_z;
    if (inductance >= (desired_delta_uH + uH_tolerance))
    {
        corrected_z += height_correction_mm;
        out_of_bounds_counter ++;
#ifdef SHAPE_MAPPING_SCANNER_DEBUG
        Serial.print("Inductance corrected height up to ");
        Serial.println(corrected_z);
#endif
    }
    if (inductance <= (desired_delta_uH - uH_tolerance))
    {
        corrected_z -= height_correction_mm;
        out_of_bounds_counter ++;
        down_counter ++;
#ifdef SHAPE_MAPPING_SCANNER_DEBUG
        Serial.print("Inductance corrected height down to ");
        Serial.println(corrected_z);
#endif
    }
    else
    {
        down_counter = 0;
    }
    move_counter ++;
    return corrected_z;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S0_Run(void)
{
    current_cycle_count = 1;
    move_counter = 0;
    out_of_bounds_counter = 0;
    down_counter = 0;
    edge_x = 0;
    edge_z = 0;
    back_in_range_count = 0;
    return S1_MOVE_ABOVE_ORIGIN;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S1_Run(void)
{
    if (transitioned_state)
    {
        SetArm(origin_x, origin_y, origin_z + DISTANCE_ABOVE_ORIGIN_START_MM,
                -90);
    }

    if (hasReachedDesiredPosition())
    {
        return S2_FIND_HEIGHT;
    }
    else
    {
        return S1_MOVE_ABOVE_ORIGIN;
    }
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S2_Run(void)
{
    if (hasReachedDesiredPosition())
    {
        SetArm(origin_x, origin_y, current_z - travel_increment_mm, -90);
    }

    if (inductance_delta > desired_delta_uH)
    {
#ifdef SHAPE_MAPPING_SCANNER_DEBUG
        Serial.print("Height found at z = ");
        Serial.print(current_z - travel_increment_mm);
        Serial.print(" current inductance = ");
        Serial.print(current_inductance);
        Serial.print(" reference = ");
        Serial.println(reference_inductance);
#endif
        return S3_HORIZONTAL_MOVE_FWD;
    }
    else
    {
#ifdef SHAPE_MAPPING_SCANNER_DEBUG
        Serial.print("Current inductance = ");
        Serial.print(current_inductance);
        Serial.print(" reference = ");
        Serial.println(reference_inductance);
#endif
        return S2_FIND_HEIGHT;
    }
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S3_Run(void)
{
    if (hasDetectedEdge())
    {
        edge_x = current_x;
        edge_z = current_z;
        Serial.print("Edge detection test start at x = ");
        Serial.print(edge_x);
        Serial.print(" z = ");
        Serial.println(edge_z);
        //return S4_HORIZ_FWD_START_TEST;
        return S9_TEST_SMALL_LOWER;
    }

    if (hasReachedDesiredPosition())
    {
        SetArm(current_x + travel_increment_mm, current_y, GetCorrectedHeight(
                    current_z, inductance_delta), -90);
    }

    return S3_HORIZONTAL_MOVE_FWD;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S4_Run(void)
{
    if (current_x >= (edge_x + 50))
    {
        return S5_HORIZ_FWD_EDGE_TEST;
    }

    if (hasReachedDesiredPosition())
    {
        SetArm(current_x + travel_increment_mm, current_y, current_z, -90);
        if (inductance_delta >= desired_delta_uH)
        {
            //back_in_range_count ++;
            //Serial.print("Edge test inductance = ");
            //Serial.print(inductance_delta);
            //Serial.print("  back in range inc to ");
            //Serial.println(back_in_range_count);
            SetArm((edge_x + origin_x) / 2.0, current_y, current_z + DISTANCE_ABOVE_ORIGIN_START_MM/3, -90);
            Serial.println("Inductance back in range, move back to horiz move");
            down_counter = 0;
            back_in_range_count = 0;
            return S7_RESTABILIZE_SCAN;
        }
    }
    //if (back_in_range_count > EDGE_DETECTION_COUNT)
    //{
    //    SetArm(edge_x - 15, current_y, current_z + DISTANCE_ABOVE_ORIGIN_START_MM/3, -90);
    //    //Serial.println("Inductance back in range, move back to horiz move");
    //    down_counter = 0;
    //    back_in_range_count = 0;
    //    return S7_RESTABILIZE_SCAN;
    //}

    return S4_HORIZ_FWD_START_TEST;

    //if (transitioned_state)
    //{
    //    Serial.println("Edge test start, moving +50 in x dir");
    //    SetArm(current_x + 50, current_y, current_z, -90);
    //    back_in_range_count = 0;
    //}

    //if (inductance_delta >= (desired_delta_uH - uH_tolerance))
    //{
    //    back_in_range_count ++;
    //    Serial.print("Edge test inductance = ");
    //    Serial.print(inductance_delta);
    //    Serial.print("  back in range inc to ");
    //    Serial.println(back_in_range_count);
    //}
    //if (back_in_range_count > EDGE_DETECTION_COUNT)
    //{
    //    Serial.println("Inductance back in range, move back to horiz move");
    //    down_counter = 0;
    //    edge_x = 0;
    //    return S3_HORIZONTAL_MOVE_FWD;
    //}
    //if (hasReachedDesiredPosition())
    //{
    //    return S5_HORIZ_FWD_EDGE_TEST;
    //}
    //return S4_HORIZ_FWD_START_TEST;
    //Serial.print("Detected Edge!");
    //return S12_SCAN_COMPLETE;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S5_Run(void)
{
    if (transitioned_state)
    {
        SetArm(current_x, current_y, current_z - 10, -90);
    }

    if (inductance_delta >= (desired_delta_uH - uH_tolerance))
    {
        Serial.println("Back in range during final down test");
        SetArm(edge_x - 15, current_y, current_z + DISTANCE_ABOVE_ORIGIN_START_MM/3, -90);
        //Serial.println("Inductance back in range, move back to horiz move");
        down_counter = 0;
        back_in_range_count = 0;
        return S7_RESTABILIZE_SCAN;
    }
    else if (hasReachedDesiredPosition())
    {
        return S8_HORIZ_FWD_EDGE_CONFIRMED;
    }
    else
    {
        return S5_HORIZ_FWD_EDGE_TEST;
    }
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S6_Run(void)
{
    if (transitioned_state)
    {
        SetArm(current_x, current_y, current_z + DISTANCE_ABOVE_ORIGIN_START_MM, -90);
    }

    if (hasReachedDesiredPosition())
    {
        return S12_SCAN_COMPLETE;
    }
    return S6_HORIZ_FWD_EDGE_RECOVERY;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S7_Run(void)
{
    if (current_x == (edge_x + origin_x) / 2.0 &&
            inductance_delta > desired_delta_uH)
    {
        //Serial.print("Scan restabilized with inductance delta = ");
        //Serial.println(inductance_delta);
        return S3_HORIZONTAL_MOVE_FWD;
    }

    if (hasReachedDesiredPosition())
    {
        if (inductance_delta <= desired_delta_uH)
        {
            SetArm((edge_x + origin_x) / 2.0, current_y, current_z - height_correction_mm, -90);
        }
    }
    return S7_RESTABILIZE_SCAN;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S8_Run(void)
{
    //Serial.print("Edge confirmed at x = ");
    //Serial.println(edge_x);
    return S6_HORIZ_FWD_EDGE_RECOVERY;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S9_Run(void)
{
    if (inductance_delta >= (desired_delta_uH - uH_tolerance))
    {
        SetArm(current_x, current_y, current_z, -90);
        return S10_INC_SMALL_LOWER_TEST;
    }
    else if (current_z <= (edge_z - 2.5))
    {
        Serial.print("Bottom of test w no sample, edge confirmed at x = ");
        Serial.println(current_x);
        return S8_HORIZ_FWD_EDGE_CONFIRMED;
    }

    //Serial.print("Lowered to z = ");
    //Serial.print(current_z);
    //Serial.print(" with inductance delta = ");
    //Serial.println(inductance_delta, 6);
    if (hasReachedDesiredPosition())
    {
        SetArm(current_x, current_y, current_z - travel_increment_mm, -90);
    }
    return S9_TEST_SMALL_LOWER;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S10_Run(void)
{
    if (transitioned_state)
    {
        SetArm(current_x + 3.5, current_y, current_z, -90);
    }

    if (hasReachedDesiredPosition())
    {
        if (current_x >= (edge_x + 14.0))
        {
            Serial.println("Shifted x 14 in edge test and still seeing sample, bailing");
            down_counter = 0;
            SetArm(edge_x - 15, current_y, current_z + DISTANCE_ABOVE_ORIGIN_START_MM/3, -90);
            return S7_RESTABILIZE_SCAN;
        }
        else
        {
            //Serial.print("Shifted x in small test over to ");
            //Serial.println(current_x);
            return S9_TEST_SMALL_LOWER;
        }
    }
    return S10_INC_SMALL_LOWER_TEST;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S12_Run(void)
{
    return S12_SCAN_COMPLETE;
}

bool ShapeMappingScanner::hasDetectedEdge()
{
    return (down_counter > EDGE_DETECTION_COUNT);
    //return ((down_counter > EDGE_DETECTION_COUNT) &&
    //        (inductance_decrease_count > EDGE_DETECTION_COUNT));
}
