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
    current_state = S0_IDLE;
    next_state = S0_IDLE;
    transitioned_state = false;
    current_scan_dir = X_FORWARD;
    edge_counter = 0;
    set_arm_error = 0;
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
        case S3_SCANNING:
            next_state = S3_Run();
            break;
        case S4_BEGIN_EDGE_TEST:
            next_state = S4_Run();
            break;
        case S5_EDGE_TEST_LOWERING:
            next_state = S5_Run();
            break;
        case S6_EDGE_CONFIRMED:
            next_state = S6_Run();
            break;
        case S7_EDGE_TEST_POSITION_INC:
            next_state = S7_Run();
            break;
        case S8_EDGE_DISPROVED:
            next_state = S8_Run();
            break;
        case S9_RESET_SCAN:
            next_state = S9_Run();
            break;
        case S10_READY_NEXT_SCAN:
            next_state = S10_Run();
            break;
        case S11_SCAN_COMPLETE:
            next_state = S11_Run();
            break;
        case S12_EDGE_OUT_OF_BOUNDS:
            next_state = S12_Run();
            break;
        case S13_SCAN_INVALID:
            next_state = S13_Run();
            break;
        case S14_VALIDATE_HEIGHT:
            next_state = S14_Run();
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

    set_arm_error = ArmController::Update(time_since_last_update_ms);
}

bool ShapeMappingScanner::isScanComplete(void)
{
    return (current_state == S11_SCAN_COMPLETE);
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
    if (inductance > (desired_delta_uH + uH_tolerance))
    {
        corrected_z += HEIGHT_CORRECTION_MM;
        out_of_bounds_counter ++;
#ifdef SHAPE_MAPPING_SCANNER_DEBUG
        Serial.print("Inductance corrected height up to ");
        Serial.println(corrected_z);
#endif
    }
    if (inductance < (desired_delta_uH - uH_tolerance))
    {
        corrected_z -= HEIGHT_CORRECTION_MM;
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
    edge_y = 0;
    edge_z = 0;
    edge_counter = 0;
    set_arm_error = 0;
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
    else if (set_arm_error)
    {
        return S12_EDGE_OUT_OF_BOUNDS;
    }
    return S1_MOVE_ABOVE_ORIGIN;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S2_Run(void)
{
    if (hasReachedDesiredPosition())
    {
        SetArm(current_x, current_y, current_z - TRAVEL_INCREMENT_MM, -90);
    }

    if (inductance_delta > desired_delta_uH)
    {
#ifdef SHAPE_MAPPING_SCANNER_DEBUG
        Serial.print("Height found at z = ");
        Serial.print(current_z - TRAVEL_INCREMENT_MM);
        Serial.print(" current inductance = ");
        Serial.print(current_inductance);
        Serial.print(" reference = ");
        Serial.println(reference_inductance);
#endif
        return S14_VALIDATE_HEIGHT;
    }
    else if (set_arm_error)
    {
        return S12_EDGE_OUT_OF_BOUNDS;
    }
#ifdef SHAPE_MAPPING_SCANNER_DEBUG
    Serial.print("Lowering - Current inductance = ");
    Serial.print(current_inductance);
    Serial.print(" reference = ");
    Serial.println(reference_inductance);
#endif
    return S2_FIND_HEIGHT;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S3_Run(void)
{
    if (transitioned_state)
    {
        down_counter = 0;
    }

    if (hasDetectedEdge())
    {
#ifdef SHAPE_MAPPING_SCANNER_DEBUG
        Serial.print("Edge detection test start at x = ");
        Serial.print(current_x);
        Serial.print(" y = ");
        Serial.print(current_y);
        Serial.print(" z = ");
        Serial.println(current_z);
#endif
        return S4_BEGIN_EDGE_TEST;
    }
    else if (set_arm_error)
    {
        return S12_EDGE_OUT_OF_BOUNDS;
    }

    if (hasReachedDesiredPosition())
    {
        SetArmScanning();
    }

    return S3_SCANNING;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S4_Run(void)
{
    edge_x = current_x;
    edge_y = current_y;
    edge_z = current_z;
    return S5_EDGE_TEST_LOWERING;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S5_Run(void)
{
    if (!isInductanceTooLow())
    {
#ifdef SHAPE_MAPPING_SCANNER_DEBUG
        Serial.println("Inductance in range during down test, move to inc position");
#endif
        return S7_EDGE_TEST_POSITION_INC;
    }
    else if (isInductanceTooLow() &&
            (current_z <= edge_z - EDGE_TEST_HEIGHT_DROP_MM))
    {
#ifdef SHAPE_MAPPING_SCANNER_DEBUG
        Serial.print("Edge confirmed at (");
        Serial.print(current_x);
        Serial.print(", ");
        Serial.print(current_y);
        Serial.print(", ");
        Serial.print(current_z);
        Serial.println(")");
#endif
        return S6_EDGE_CONFIRMED;
    }
    else if (set_arm_error)
    {
        return S12_EDGE_OUT_OF_BOUNDS;
    }

    if (hasReachedDesiredPosition())
    {
        SetArm(current_x, current_y, current_z - TRAVEL_INCREMENT_MM, -90);
    }
    return S5_EDGE_TEST_LOWERING;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S6_Run(void)
{
    if (transitioned_state)
    {
        MapEdge();
        edge_counter ++;
        SetArm(current_x, current_y, current_z + HEIGHT_SAFETY_DIST_MM, -90);
    }

    if (hasReachedDesiredPosition())
    {
        return S10_READY_NEXT_SCAN;
    }
    return S6_EDGE_CONFIRMED;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S7_Run(void)
{
    if (transitioned_state)
    {
        SetArmEdgeTestPosition();
    }

    if (hasReachedDesiredPosition())
    {
        if (hasPassedEndOfEdgeTestRange())
        {
#ifdef SHAPE_MAPPING_SCANNER_DEBUG
            Serial.println("Full horiz range reached, edge disproved");
#endif
            return S8_EDGE_DISPROVED;
        }
        else
        {
            return S5_EDGE_TEST_LOWERING;
        }
    }
    else if (set_arm_error)
    {
        return S12_EDGE_OUT_OF_BOUNDS;
    }
    return S7_EDGE_TEST_POSITION_INC;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S8_Run(void)
{
    if (transitioned_state)
    {
        SetArm(current_x, current_y, current_z + HEIGHT_SAFETY_DIST_MM, -90);
    }

    if (hasReachedDesiredPosition())
    {
        return S9_RESET_SCAN;
    }
    return S8_EDGE_DISPROVED;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S9_Run(void)
{
    if (transitioned_state)
    {
        SetArmResettingScan();
    }

    if (hasReachedDesiredPosition())
    {
        return S2_FIND_HEIGHT;
    }
    return S9_RESET_SCAN;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S10_Run(void)
{
    if (transitioned_state)
    {
        switch (current_scan_dir)
        {
            case X_FORWARD:
                current_scan_dir = X_BACKWARD;
                break;
            case X_BACKWARD:
                current_scan_dir = Y_FORWARD;
                break;
            case Y_FORWARD:
                current_scan_dir = Y_BACKWARD;
                break;
            case Y_BACKWARD:
                current_scan_dir = X_FORWARD;
                break;
            default:
                break;
        }
        SetArmWithNextScanOrigin();
    }

    if (hasReachedDesiredPosition())
    {
        if (edge_counter >= NUM_EDGES)
        {
            return S11_SCAN_COMPLETE;
        }
        else
        {
            return S2_FIND_HEIGHT;
        }
    }
    return S10_READY_NEXT_SCAN;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S11_Run(void)
{
    if (transitioned_state)
    {
//#ifdef SHAPE_MAPPING_SCANNER_DEBUG
        Serial.println("Shape edges:");
        for (uint8_t i = 0; i < NUM_EDGES; i ++)
        {
            Serial.print("    ");
            Serial.print(edge_map[i].x_coordinate);
            Serial.print(" , ");
            Serial.println(edge_map[i].y_coordinate);
        }
//#endif
    }
    return S11_SCAN_COMPLETE;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S12_Run(void)
{
    if (transitioned_state)
    {
        MapOutOfBoundsEdge();
        edge_counter ++;
        SetArm(current_x, current_y, current_z + HEIGHT_SAFETY_DIST_MM, -90);
    }

    if (hasReachedDesiredPosition())
    {
        return S10_READY_NEXT_SCAN;
    }
    return S12_EDGE_OUT_OF_BOUNDS;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S13_Run(void)
{
    return S13_SCAN_INVALID;
}

ShapeMappingScanner::scan_state ShapeMappingScanner::S14_Run(void)
{
    if (transitioned_state)
    {
        in_range_counter = 0;
    }
    in_range_counter ++;
#ifdef SHAPE_MAPPING_SCANNER_DEBUG
    Serial.print("Validating height, inductance delta = ");
    Serial.println(inductance_delta, 4);
#endif

    if (isInductanceTooLow())
    {
#ifdef SHAPE_MAPPING_SCANNER_DEBUG
        Serial.println("Inductance too low, move back to find height");
#endif
        return S2_FIND_HEIGHT;
    }
    else if (in_range_counter >= HEIGHT_VALIDATION_COUNT)
    {
#ifdef SHAPE_MAPPING_SCANNER_DEBUG
        Serial.println("Counter reached, height validated, move to scan");
#endif
        return S3_SCANNING;
    }
    return S14_VALIDATE_HEIGHT;
}

bool ShapeMappingScanner::hasDetectedEdge(void)
{
    return (down_counter > EDGE_DETECTION_COUNT);
}

bool ShapeMappingScanner::isInductanceTooLow(void)
{
    return (inductance_delta < (desired_delta_uH - uH_tolerance));
}

bool ShapeMappingScanner::hasPassedEndOfEdgeTestRange(void)
{
    switch (current_scan_dir)
    {
        case X_FORWARD:
            return (current_x > (edge_x + EDGE_TEST_HORIZ_RANGE_MM));
            break;
        case X_BACKWARD:
            return (current_x < (edge_x - EDGE_TEST_HORIZ_RANGE_MM));
            break;
        case Y_FORWARD:
            return (current_y > (edge_y + EDGE_TEST_HORIZ_RANGE_MM));
            break;
        case Y_BACKWARD:
            return (current_y < (edge_y - EDGE_TEST_HORIZ_RANGE_MM));
            break;
        default:
            return true;
            break;
    }
}

void ShapeMappingScanner::SetArmScanning(void)
{
    switch (current_scan_dir)
    {
        case X_FORWARD:
            SetArm(current_x + TRAVEL_INCREMENT_MM, current_y,
                    GetCorrectedHeight(current_z, inductance_delta), -90);
            break;
        case X_BACKWARD:
            SetArm(current_x - TRAVEL_INCREMENT_MM, current_y,
                    GetCorrectedHeight(current_z, inductance_delta), -90);
            break;
        case Y_FORWARD:
            SetArm(current_x, current_y + TRAVEL_INCREMENT_MM,
                    GetCorrectedHeight(current_z, inductance_delta), -90);
            break;
        case Y_BACKWARD:
            SetArm(current_x, current_y - TRAVEL_INCREMENT_MM,
                    GetCorrectedHeight(current_z, inductance_delta), -90);
            break;
        default:
            break;
    }
}

void ShapeMappingScanner::SetArmEdgeTestPosition(void)
{
    switch (current_scan_dir)
    {
        case X_FORWARD:
            SetArm(current_x + EDGE_TEST_HORIZ_INC_MM, current_y, current_z,
                    -90);
            break;
        case X_BACKWARD:
            SetArm(current_x - EDGE_TEST_HORIZ_INC_MM, current_y, current_z,
                    -90);
            break;
        case Y_FORWARD:
            SetArm(current_x, current_y + EDGE_TEST_HORIZ_INC_MM, current_z,
                    -90);
            break;
        case Y_BACKWARD:
            SetArm(current_x, current_y - EDGE_TEST_HORIZ_INC_MM, current_z,
                    -90);
            break;
        default:
            break;
    }
}

void ShapeMappingScanner::SetArmResettingScan(void)
{
    switch (current_scan_dir)
    {
        case X_FORWARD:
        case X_BACKWARD:
            SetArm((edge_x + origin_x) / 2.0, current_y, current_z, -90);
            break;
        case Y_FORWARD:
        case Y_BACKWARD:
            SetArm(current_x, (edge_y + origin_y) / 2.0, current_z, -90);
            break;
        default:
            break;
    }
}

void ShapeMappingScanner::SetArmWithNextScanOrigin(void)
{
    switch (edge_counter)
    {
        case 0:
        case 1:
        case 2:
        case 3:
            //first scans all from origin
            SetArm(origin_x, origin_y, origin_z + DISTANCE_ABOVE_ORIGIN_START_MM,
                    -90);
            break;
        case 4:
        case 5:
            //second x
            SetArm(origin_x, (0.75 * edge_map[2].y_coordinate) +
                    (0.25 * edge_map[3].y_coordinate),
                    origin_z + DISTANCE_ABOVE_ORIGIN_START_MM, -90);
            break;
        case 6:
        case 7:
            //second y
            SetArm((0.75 * edge_map[0].x_coordinate) +
                    (0.25 * edge_map[1].x_coordinate), origin_y,
                    origin_z + DISTANCE_ABOVE_ORIGIN_START_MM, -90);
            break;
        case 8:
        case 9:
            //third x
            SetArm(origin_x, (0.25 * edge_map[2].y_coordinate) +
                    (0.75 * edge_map[3].y_coordinate),
                    origin_z + DISTANCE_ABOVE_ORIGIN_START_MM, -90);
            break;
            break;
        case 10:
        case 11:
            //third y
            SetArm((0.25 * edge_map[0].x_coordinate) +
                    (0.75 * edge_map[1].x_coordinate), origin_y,
                    origin_z + DISTANCE_ABOVE_ORIGIN_START_MM, -90);
            break;
            break;
        default:
            SetArm(origin_x, origin_y, origin_z + DISTANCE_ABOVE_ORIGIN_START_MM,
                    -90);
            break;
    }
}

void ShapeMappingScanner::MapEdge(void)
{
    switch (current_scan_dir)
    {
        case X_FORWARD:
            edge_map[edge_counter].x_coordinate =
                current_x - SENSOR_COIL_RADIUS_MM;;
            edge_map[edge_counter].y_coordinate = current_y;
            break;
        case X_BACKWARD:
            edge_map[edge_counter].x_coordinate =
                current_x + SENSOR_COIL_RADIUS_MM;
            edge_map[edge_counter].y_coordinate = current_y;
            break;
        case Y_FORWARD:
            edge_map[edge_counter].x_coordinate = current_x;
            edge_map[edge_counter].y_coordinate =
                current_y - SENSOR_COIL_RADIUS_MM;
            break;
        case Y_BACKWARD:
            edge_map[edge_counter].x_coordinate = current_x;
            edge_map[edge_counter].y_coordinate =
                current_y + SENSOR_COIL_RADIUS_MM;
            break;
        default:
            break;
    }
}

void ShapeMappingScanner::MapOutOfBoundsEdge(void)
{
    switch (current_scan_dir)
    {
        case X_FORWARD:
        case X_BACKWARD:
            edge_map[edge_counter].x_coordinate = -1;
            edge_map[edge_counter].y_coordinate = current_y;
            break;
        case Y_FORWARD:
        case Y_BACKWARD:
            edge_map[edge_counter].x_coordinate = current_x;
            edge_map[edge_counter].y_coordinate = -1;
            break;
        default:
            break;
    }
}
