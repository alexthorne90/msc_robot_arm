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
    float current_inductance;
    metal_detector.Update();
    current_inductance = metal_detector.GetCh0InductanceuH();

    ArmController::Update(time_since_last_update_ms);
}
