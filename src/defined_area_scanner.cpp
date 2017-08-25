/**
 * @file defined_area_scanner.cpp
 * @author Alex Thorne
 * @version 1.0
 */

#include "defined_area_scanner.h"

//#define DEFINED_AREA_SCANNER_DEBUG

DefinedAreaScanner::DefinedAreaScanner(void) : MetalScanner()
{
	//initialize
    horizontal_desired_travel_mm = DEFAULT_HORIZONTAL_DESIRED_TRAVEL_MM;
    depth_desired_travel_mm = DEFAULT_DEPTH_DESIRED_TRAVEL_MM;
    depth_scan_increment_mm = DEFAULT_DEPTH_SCAN_INCREMENT_MM;
    travel_increment_mm = DEFAULT_TRAVEL_INCREMENT_MM;
    height_correction_mm = DEFAULT_HEIGHT_CORRECTION_MM;
    current_state = S0_IDLE;
    next_state = S0_IDLE;
    transitioned_state = false;
}

uint8_t DefinedAreaScanner::SetDesiredHorizontalTravelMM(float horizontal_travel)
{
    if (horizontal_travel > (X_MAX_BOUNDARY - X_MIN_BOUNDARY) ||
            horizontal_travel < 0)
        return 1;
    horizontal_desired_travel_mm = horizontal_travel;
    return 0;
}

uint8_t DefinedAreaScanner::SetDesiredDepthTravelMM(float depth_travel)
{
    if (depth_travel > (Y_MAX_BOUNDARY - Y_MIN_BOUNDARY) ||
            depth_travel < 0)
        return 1;
    depth_desired_travel_mm = depth_travel;
    return 0;
}

uint8_t DefinedAreaScanner::SetDepthScanIncrementMM(float depth_increment)
{
    if (depth_increment > (Y_MAX_BOUNDARY - Y_MIN_BOUNDARY) ||
            depth_increment < 0)
        return 1;
    depth_scan_increment_mm = depth_increment;
    return 0;
}

uint8_t DefinedAreaScanner::SetTravelIncrementMM(float travel_increment)
{
    if (travel_increment_mm > MAX_TRAVEL_INCREMENT_MM || travel_increment_mm < 0)
        return 1;
    travel_increment_mm = travel_increment;
    return 0;
}

uint8_t DefinedAreaScanner::SetHeightCorrectionMM(float height_correction)
{
    if (height_correction > MAX_HEIGHT_ADJUSTMENT_MM || height_correction_mm < 0)
        return 1;
    height_correction_mm = height_correction;
    return 0;
}

void DefinedAreaScanner::Update(uint16_t time_since_last_update_ms)
{
    metal_detector.Update();
    current_inductance = metal_detector.GetCh0InductanceuH();
    inductance_delta = reference_inductance - current_inductance;
    current_x = GetCurrentX();
    current_y = GetCurrentY();
    current_z = GetCurrentZ();

    if (!isScanAreaValid())
        return;

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
        case S4_DEPTH_MOVE:
            next_state = S4_Run();
            break;
        case S5_HORIZONTAL_MOVE_BACK:
            next_state = S5_Run();
            break;
        case S6_CYCLE_END:
            next_state = S6_Run();
            break;
        case S7_SCAN_FINISHED:
            next_state = S7_Run();
            break;
        default:
            break;
    }
    if (current_state != next_state)
    {
#ifdef DEFINED_AREA_SCANNER_DEBUG
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

bool DefinedAreaScanner::isScanComplete(void)
{
    return (current_state == S7_SCAN_FINISHED);
}

void DefinedAreaScanner::ResetScan(void)
{
    current_state = S0_IDLE;
    next_state = S0_IDLE;
}

float DefinedAreaScanner::GetOutOfBoundsPercentage(void)
{
    if (move_counter <= 0)
    {
        return 0;
    }
    return ((float)out_of_bounds_counter / (float)move_counter * 100.0);
}

// Private functions ***********************************************************
float DefinedAreaScanner::GetCorrectedHeight(float current_z, float inductance)
{
    float corrected_z = current_z;
    if (inductance >= (desired_delta_uH + uH_tolerance))
    {
        corrected_z += height_correction_mm;
        out_of_bounds_counter ++;
#ifdef DEFINED_AREA_SCANNER_DEBUG
        Serial.print("Inductance corrected height up to ");
        Serial.println(corrected_z);
#endif
    }
    if (inductance <= (desired_delta_uH - uH_tolerance))
    {
        corrected_z -= height_correction_mm;
        out_of_bounds_counter ++;
#ifdef DEFINED_AREA_SCANNER_DEBUG
        Serial.print("Inductance corrected height down to ");
        Serial.println(corrected_z);
#endif
    }
    move_counter ++;
    return corrected_z;
}

DefinedAreaScanner::scan_state DefinedAreaScanner::S0_Run(void)
{
    current_cycle_count = 1;
    move_counter = 0;
    out_of_bounds_counter = 0;
    return S1_MOVE_ABOVE_ORIGIN;
}

DefinedAreaScanner::scan_state DefinedAreaScanner::S1_Run(void)
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

DefinedAreaScanner::scan_state DefinedAreaScanner::S2_Run(void)
{
    if (hasReachedDesiredPosition())
    {
        SetArm(origin_x, origin_y, current_z - travel_increment_mm, -90);
    }

    if (inductance_delta > desired_delta_uH)
    {
#ifdef DEFINED_AREA_SCANNER_DEBUG
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
#ifdef DEFINED_AREA_SCANNER_DEBUG
        Serial.print("Current inductance = ");
        Serial.print(current_inductance);
        Serial.print(" reference = ");
        Serial.println(reference_inductance);
#endif
        return S2_FIND_HEIGHT;
    }
}

DefinedAreaScanner::scan_state DefinedAreaScanner::S3_Run(void)
{
    if (current_x >= (horizontal_desired_travel_mm + origin_x))
    {
#ifdef DEFINED_AREA_SCANNER_DEBUG
        Serial.print("Horizontal fwd move completed at x = ");
        Serial.print(current_x);
        Serial.print(" y = ");
        Serial.print(current_y);
        Serial.print(" z = ");
        Serial.println(current_z);
#endif
        return S4_DEPTH_MOVE;
    }

    if (hasReachedDesiredPosition())
    {
        SetArm(current_x + travel_increment_mm, current_y, GetCorrectedHeight(
                    current_z, inductance_delta), -90);
    }

    return S3_HORIZONTAL_MOVE_FWD;
}

DefinedAreaScanner::scan_state DefinedAreaScanner::S4_Run(void)
{
    if (current_y >= ((depth_scan_increment_mm * current_cycle_count) +
                origin_y))
    {
#ifdef DEFINED_AREA_SCANNER_DEBUG
        Serial.print("Depth move completed at x = ");
        Serial.print(current_x);
        Serial.print(" y = ");
        Serial.print(current_y);
        Serial.print(" z = ");
        Serial.println(current_z);
#endif
        return S6_CYCLE_END;
    }

    if (hasReachedDesiredPosition())
    {
        SetArm(current_x, current_y + travel_increment_mm, GetCorrectedHeight(
                    current_z, inductance_delta), -90);
    }

    return S4_DEPTH_MOVE;
}

DefinedAreaScanner::scan_state DefinedAreaScanner::S5_Run(void)
{
    if (current_x <= origin_x)
    {
#ifdef DEFINED_AREA_SCANNER_DEBUG
        Serial.print("Horizontal back move completed at x = ");
        Serial.print(current_x);
        Serial.print(" y = ");
        Serial.print(current_y);
        Serial.print(" z = ");
        Serial.println(current_z);
#endif
        return S4_DEPTH_MOVE;
    }

    if (hasReachedDesiredPosition())
    {
        SetArm(current_x - travel_increment_mm, current_y, GetCorrectedHeight(
                    current_z, inductance_delta), -90);
    }

    return S5_HORIZONTAL_MOVE_BACK;
}

DefinedAreaScanner::scan_state DefinedAreaScanner::S6_Run(void)
{
    if (transitioned_state)
    {
        current_cycle_count ++;
    }

    if ((current_y < (depth_desired_travel_mm + origin_y)) &&
            (current_x >= (horizontal_desired_travel_mm + origin_x)))
    {
        return S5_HORIZONTAL_MOVE_BACK;
    }
    else if ((current_y < (depth_desired_travel_mm + origin_y)) &&
            (current_x <= origin_x))
    {
        return S3_HORIZONTAL_MOVE_FWD;
    }
    else if (current_y >= (depth_desired_travel_mm + origin_y))
    {
        return S7_SCAN_FINISHED;
    }
    else
    {
        return S6_CYCLE_END;
    }
}

DefinedAreaScanner::scan_state DefinedAreaScanner::S7_Run(void)
{
#ifdef DEFINED_AREA_SCANNER_DEBUG
    if (transitioned_state)
    {
        Serial.println("Scan complete!");
    }
#endif
    return S7_SCAN_FINISHED;
}

bool DefinedAreaScanner::isScanAreaValid(void)
{
    if ((origin_x + horizontal_desired_travel_mm) > X_MAX_BOUNDARY)
        return false;
    if ((origin_y + depth_desired_travel_mm) > Y_MAX_BOUNDARY)
        return false;
    return true;
}
