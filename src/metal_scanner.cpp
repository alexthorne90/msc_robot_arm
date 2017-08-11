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

void MetalScanner::Update(uint16_t time_since_last_update_ms)
{
    float current_inductance;
    metal_detector.Update();
    current_inductance = metal_detector.GetCh0InductanceuH();

    ArmController::Update(time_since_last_update_ms);
}
