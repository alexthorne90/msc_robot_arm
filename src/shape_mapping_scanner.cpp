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
}

bool ShapeMappingScanner::isScanComplete(void)
{
    return false;
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
#ifdef SHAPE_MAPPING_SCANNER_DEBUG
        Serial.print("Inductance corrected height down to ");
        Serial.println(corrected_z);
#endif
    }
    move_counter ++;
    return corrected_z;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S0_Run(void)
{
    current_cycle_count = 1;
    move_counter = 0;
    out_of_bounds_counter = 0;
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
        return S0_IDLE;
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
