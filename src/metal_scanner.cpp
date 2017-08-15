/**
 * @file metal_scanner_lib.cpp
 * @author Alex Thorne
 * @version 1.0
 */

#include "metal_scanner.h"

//#define METAL_SCANNER_DEBUG

MetalScanner::MetalScanner(void) : ArmController()
{
	//initialize
    reference_inductance = 0;
    desired_delta_uH = DEFAULT_DELTA_uH;
    uH_tolerance = DEFAULT_uH_TOLERANCE;
    horizontal_desired_travel_mm = DEFAULT_HORIZONTAL_DESIRED_TRAVEL_MM;
    depth_desired_travel_mm = DEFAULT_DEPTH_DESIRED_TRAVEL_MM;
    depth_scan_increment_mm = DEFAULT_DEPTH_SCAN_INCREMENT_MM;
    current_state = S0_IDLE;
    next_state = S0_IDLE;
    transitioned_state = false;
}

void MetalScanner::HardwareSetup(void)
{
    metal_detector.ConnectToSensor();
    metal_detector.ConfigureSensor();
    AttachMotors();
}

void MetalScanner::SetReferenceInductance(void)
{
    metal_detector.Update();
    reference_inductance = metal_detector.GetCh0InductanceuH();
}

uint8_t MetalScanner::SetScanOrigin(float x, float y, float z)
{
    origin_x = x;
    origin_y = y;
    origin_z = z;
    return 0;
}

uint8_t MetalScanner::SetDesiredInductanceDelta(float inductance)
{
    desired_delta_uH = inductance;
    return 0;
}

uint8_t MetalScanner::SetInductanceDeltaTolerance(float tolerance)
{
    uH_tolerance = tolerance;
    return 0;
}

uint8_t MetalScanner::SetDesiredHorizontalTravelMM(float horizontal_travel)
{
    horizontal_desired_travel_mm = horizontal_travel;
    return 0;
}

uint8_t MetalScanner::SetDesiredDepthTravelMM(float depth_travel)
{
    depth_desired_travel_mm = depth_travel;
    return 0;
}

uint8_t MetalScanner::SetDepthScanIncrement(float depth_increment)
{
    depth_scan_increment_mm = depth_increment;
    return 0;
}

void MetalScanner::Update(uint16_t time_since_last_update_ms)
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
#ifdef METAL_SCANNER_DEBUG
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

bool MetalScanner::isScanComplete(void)
{
    return (current_state == S7_SCAN_FINISHED);
}

void MetalScanner::ResetScan(void)
{
    current_state = S0_IDLE;
    next_state = S0_IDLE;
}

// Private functions ***********************************************************
float MetalScanner::GetCorrectedHeight(float current_z, float inductance)
{
    float corrected_z = current_z;
    if (inductance >= (desired_delta_uH + uH_tolerance))
    {
        corrected_z += 1.0;
#ifdef METAL_SCANNER_DEBUG
        Serial.print("Inductance corrected height up to ");
        Serial.println(corrected_z);
#endif
    }
    if (inductance <= (desired_delta_uH - uH_tolerance))
    {
        corrected_z -= 1.0;
#ifdef METAL_SCANNER_DEBUG
        Serial.print("Inductance corrected height down to ");
        Serial.println(corrected_z);
#endif
    }
    return corrected_z;
}

MetalScanner::scan_state MetalScanner::S0_Run(void)
{
    current_cycle_count = 1;
    return S1_MOVE_ABOVE_ORIGIN;
}

MetalScanner::scan_state MetalScanner::S1_Run(void)
{
    if (transitioned_state)
    {
        SetArm(origin_x, origin_y, origin_z + 60.0, -90);
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

MetalScanner::scan_state MetalScanner::S2_Run(void)
{
    if (hasReachedDesiredPosition())
    {
        SetArm(origin_x, origin_y, current_z - 1.0, -90);
    }

    if (inductance_delta > desired_delta_uH)
    {
#ifdef METAL_SCANNER_DEBUG
        Serial.print("Height found at z = ");
        Serial.print(current_z - 1.0);
        Serial.print(" current inductance = ");
        Serial.print(current_inductance);
        Serial.print(" reference = ");
        Serial.println(reference_inductance);
#endif
        return S3_HORIZONTAL_MOVE_FWD;
    }
    else
    {
        return S2_FIND_HEIGHT;
    }
}

MetalScanner::scan_state MetalScanner::S3_Run(void)
{
    if (current_x >= (horizontal_desired_travel_mm + origin_x))
    {
#ifdef METAL_SCANNER_DEBUG
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
        SetArm(current_x + 1.0, current_y, GetCorrectedHeight(
                    current_z, inductance_delta), -90);
    }

    return S3_HORIZONTAL_MOVE_FWD;
}

MetalScanner::scan_state MetalScanner::S4_Run(void)
{
    if (current_y >= ((depth_scan_increment_mm * current_cycle_count) +
                origin_y))
    {
#ifdef METAL_SCANNER_DEBUG
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
        SetArm(current_x, current_y + 1.0, GetCorrectedHeight(
                    current_z, inductance_delta), -90);
    }

    return S4_DEPTH_MOVE;
}

MetalScanner::scan_state MetalScanner::S5_Run(void)
{
    if (current_x <= origin_x)
    {
#ifdef METAL_SCANNER_DEBUG
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
        SetArm(current_x - 1.0, current_y, GetCorrectedHeight(
                    current_z, inductance_delta), -90);
    }

    return S5_HORIZONTAL_MOVE_BACK;
}

MetalScanner::scan_state MetalScanner::S6_Run(void)
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

MetalScanner::scan_state MetalScanner::S7_Run(void)
{
#ifdef METAL_SCANNER_DEBUG
    if (transitioned_state)
    {
        Serial.println("Scan complete!");
    }
#endif
    return S7_SCAN_FINISHED;
}
