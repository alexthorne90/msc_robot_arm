/**
 * @file metal_scanner.cpp
 * @author Alex Thorne
 * @version 1.0
 */

#include "metal_scanner.h"

MetalScanner::MetalScanner(void) : ArmController()
{
	//initialize
    reference_inductance = 0;
    origin_x = (X_MAX_BOUNDARY + X_MIN_BOUNDARY) / 2;
    origin_y = (Y_MAX_BOUNDARY + Y_MIN_BOUNDARY) / 2;
    origin_z = (Z_MAX_BOUNDARY + Z_MIN_BOUNDARY) / 2;
    desired_delta_uH = DEFAULT_DELTA_uH;
    uH_tolerance = DEFAULT_uH_TOLERANCE;
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
    if (x > X_MAX_BOUNDARY || x < X_MIN_BOUNDARY)
        return 1;
    if (y > Y_MAX_BOUNDARY || y < Y_MIN_BOUNDARY)
        return 1;
    if (z > Z_MAX_BOUNDARY || z < Z_MIN_BOUNDARY)
        return 1;
    origin_x = x;
    origin_y = y;
    origin_z = z;
    return 0;
}

uint8_t MetalScanner::SetDesiredInductanceDelta(float inductance)
{
    if (inductance < 0 || inductance > 3.0)
        return 1;
    desired_delta_uH = inductance;
    return 0;
}

uint8_t MetalScanner::SetInductanceDeltaTolerance(float tolerance)
{
    if (tolerance < 0)
        return 1;
    uH_tolerance = tolerance;
    return 0;
}
