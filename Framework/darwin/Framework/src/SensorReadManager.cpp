#include <iomanip>
#include <vector>
#include <algorithm>
#include "FSR.h"
#include "CM730.h"
#include "MotionStatus.h"
#include "SensorReadManager.h"
#include "SensorReadDescriptor.h"
#include "CompareSensorReadDescriptors.h"

using namespace Robot;

// Provide definition of constants here to prevent linker error.
// (am not sure why this is necessary)
const int Robot::FSR::ID_R_FSR;
const int Robot::FSR::ID_L_FSR;

// Error flags returned by sensor + servo reads/commands.
#define SENSOR_ERROR_NONE               (0x0000)
#define SENSOR_ERROR_NO_RESPONSE        (-1)
#define SENSOR_ERROR_FLAG_INPUT_VOLTAGE (0x0001)
#define SENSOR_ERROR_FLAG_ANGLE_LIMIT   (0x0002)
#define SENSOR_ERROR_FLAG_OVERHEATING   (0x0004)
#define SENSOR_ERROR_FLAG_RANGE         (0x0008)
#define SENSOR_ERROR_FLAG_CHECKSUM      (0x0010)
#define SENSOR_ERROR_FLAG_OVERLOAD      (0x0020)
#define SENSOR_ERROR_FLAG_INSTRUCTION   (0x0040)

// Duplicated from CM730.cpp. These should be constants.
#define ID                  (2)
#define LENGTH              (3)
#define INSTRUCTION         (4)
#define ERRBIT              (4)
#define PARAMETER           (5)
#define DEFAULT_BAUDNUMBER  (1)
#define INST_PING       (1)
#define INST_READ       (2)
#define INST_WRITE      (3)
#define INST_REG_WRITE  (4)
#define INST_ACTION     (5)
#define INST_RESET      (6)
#define INST_SYNC_WRITE (131)   // 0x83
#define INST_BULK_READ  (146)   // 0x92


SensorReadManager::SensorReadManager()
{
    // Should consider pinging things sometime:
    // Ping(CM730::ID_CM, NULL) == SUCCESS

    // Create the read descriptors:

    //   - CM730:
    SensorReadDescriptor* cm_read = new SensorReadDescriptor();
    cm_read->set_sensor_id(CM730::ID_CM);
    cm_read->set_start_address(CM730::P_BUTTON);
    cm_read->set_num_bytes(20);
    descriptor_list_.push_back(cm_read);

    //   - Servo motors:
    for(int servo_id = 1; servo_id < JointData::NUMBER_OF_JOINTS; ++servo_id)
    {
        SensorReadDescriptor* servo_read = new SensorReadDescriptor();
        servo_read->set_sensor_id(servo_id);
        servo_read->set_start_address(MX28::P_PRESENT_POSITION_L);
        servo_read->set_num_bytes(2);
        descriptor_list_.push_back(servo_read);
    }

//    // //   - Force-sensitive resistors:
//    SensorReadDescriptor* fsr_l_read = new SensorReadDescriptor();
//    fsr_l_read->set_sensor_id(FSR::ID_L_FSR);
//    fsr_l_read->set_start_address(FSR::P_FSR1_L);
//    fsr_l_read->set_num_bytes(10);
//    descriptor_list_.push_back(fsr_l_read);

//    SensorReadDescriptor* fsr_r_read = new SensorReadDescriptor();
//    fsr_r_read->set_sensor_id(Robot::FSR::ID_R_FSR);
//    fsr_r_read->set_start_address(FSR::P_FSR1_L);
//    fsr_r_read->set_num_bytes(10);
//    descriptor_list_.push_back(fsr_r_read);


    // Populate the map and the heap with the same descriptors:
    for (std::vector<SensorReadDescriptor*>::iterator it = descriptor_list_.begin();
         it != descriptor_list_.end(); ++it)
    {
        SensorReadDescriptor* sensor_read = *it;
        descriptor_heap_.push_back(sensor_read);
        descriptor_map_[sensor_read->sensor_id()] = sensor_read;
    }
}

SensorReadManager::~SensorReadManager()
{
    // Delete the descriptors
    for (std::vector<SensorReadDescriptor*>::iterator it = descriptor_list_.begin();
         it != descriptor_list_.end(); ++it)
    {
        SensorReadDescriptor* sensor_read = *it;
        delete sensor_read;
    }
}

// SensorReadDescriptor& SensorReadManager::operator[](const int sensor_id)
// {
//     return *(descriptor_map_[sensor_id]);
// }

SensorReadDescriptor* SensorReadManager::GetDescriptorById(int sensor_id)
{
    if(descriptor_map_.end() == descriptor_map_.find(sensor_id))
    {
        // If the sensor_id is not found
        std::cout 
            << __PRETTY_FUNCTION__ 
            << ": Sensor " << sensor_id << " not found."
            << std::endl;
    }

    return descriptor_map_[sensor_id];
}

bool SensorReadManager::ProcessBulkReadErrors(
    int bulk_read_error_code,
    BulkReadData* bulk_read_data)
{
    // A flag to indicate whether the bulk read must be repeated
    // (i.e. it is set to true if a significant error occurs during the read)
    bool error_occurred = false;

    if(bulk_read_error_code != Robot::CM730::SUCCESS)
    {
        // Check for servo read errors: (and also other sensors)
        // Note: Possible error flags are:
        //  { SENSOR_ERROR_NONE, SENSOR_ERROR_FLAG_INPUT_VOLTAGE,
        //    SENSOR_ERROR_FLAG_ANGLE_LIMIT, SENSOR_ERROR_FLAG_OVERHEATING,
        //    SENSOR_ERROR_FLAG_RANGE, SENSOR_ERROR_FLAG_CHECKSUM,
        //    SENSOR_ERROR_FLAG_OVERLOAD, SENSOR_ERROR_FLAG_INSTRUCTION }
        // std::cout    
        //         << std::endl
        //         // << __PRETTY_FUNCTION__ << ": "
        //         << "DS::CFHC()" << ": "
        //         << "BULK READ ERROR: "
        //         << Robot::CM730::getTxRxErrorString(bulk_read_error_code)
        //         << std::endl;

        bool sensor_read_error = CheckSensorsBulkReadErrors(bulk_read_data);

        // Decide whether to repeat the read based on errors returned:
        error_occurred = sensor_read_error;
    }
    else
    {
        UpdateSensorResponseRates(SENSOR_ERROR_NONE);
        // PrintSensorResponseRates();
    }

    return error_occurred;
}

void SensorReadManager::MakeBulkReadPacket(unsigned char* bulk_read_tx_packet_)
{
    const int kDataStart = (PARAMETER) + 1;

    // Note: descriptor_heap_ doesn't need to be a heap, and, whatever it is,
    //       it should probably only be accessed through an iterator.
    std::make_heap(
        descriptor_heap_.begin(), 
        descriptor_heap_.end(),
        CompareSensorReadDescriptors());
    std::sort_heap(
        descriptor_heap_.begin(),
        descriptor_heap_.end(),
        CompareSensorReadDescriptors());

    int pos = 0;
    for(int i = 0; i < (int)descriptor_heap_.size(); i++)
    {
        SensorReadDescriptor* sensor_read = descriptor_heap_[i];
        // SensorReadDescriptor* sensor_read = descriptor_heap_[(descriptor_heap_.size() - 1) - i];
        // SensorReadDescriptor* sensor_read = descriptor_heap_.front();
        // std::pop_heap(
        //     descriptor_heap_.begin(),
        //     descriptor_heap_.end() - i,
        //     CompareSensorReadDescriptors());
        bulk_read_tx_packet_[kDataStart + pos++] = sensor_read->num_bytes();
        bulk_read_tx_packet_[kDataStart + pos++] = sensor_read->sensor_id();
        bulk_read_tx_packet_[kDataStart + pos++] = sensor_read->start_address();
    }

    bulk_read_tx_packet_[ID]          = (unsigned char)CM730::ID_BROADCAST;
    bulk_read_tx_packet_[LENGTH]      = 3 + pos;
    bulk_read_tx_packet_[INSTRUCTION] = INST_BULK_READ;
    bulk_read_tx_packet_[PARAMETER]   = (unsigned char)0x0;
}

bool SensorReadManager::CheckSensorsBulkReadErrors(BulkReadData* bulk_read_data)
{
    bool significant_error_occured = false;
    bool error_occured = false;

    for (std::vector<SensorReadDescriptor*>::iterator it = descriptor_heap_.begin();
         it != descriptor_heap_.end(); ++it)
    {
        SensorReadDescriptor* sensor_read = *it;
        
        // If no errors have yet been detected
        if(!error_occured)
        {
            int sensor_error_code = GetSensorBulkReadErrors(sensor_read,
                                                            bulk_read_data);

            sensor_read->UpdateResponseRate(sensor_error_code);

            significant_error_occured |= CheckSensorBulkReadErrors(
                sensor_read,
                sensor_error_code);

            // Note: Even if the error detected was not significant,
            //       all read errors after the first are always bogus
            //       so we must stop checking errors here.
            // break;
            error_occured |= (sensor_error_code == SENSOR_ERROR_NO_RESPONSE);
        }

        // PrintSensorResponseRate(sensor_read->sensor_id());
    }

    // #warning Must return actual error!
    // std::cout   << __PRETTY_FUNCTION__ << " - " 
    //             << "DEBUG: return false;"
    //             << std::endl;
    return significant_error_occured;
}

int SensorReadManager::GetSensorBulkReadErrors(
    SensorReadDescriptor* sensor_read,
    BulkReadData* bulk_read_data)
{
    int sensor_id = sensor_read->sensor_id();
    return bulk_read_data[sensor_id].error;
}

bool SensorReadManager::CheckSensorBulkReadErrors(
    SensorReadDescriptor* sensor_read,
    int sensor_error_code)
{
    bool error_is_significant = false;

    int consecutive_errors = sensor_read->consecutive_errors();
    int sensor_id = sensor_read->sensor_id();

    if(sensor_error_code == SENSOR_ERROR_NO_RESPONSE) {
        // If the error occurs very often, we should stop reporting it,
        // since repeating the bulk read indefinitely will freeze the robot.
        // i.e. only report errors in sensors with HIGH response rates.
        // if(response_rate > 0.5)
        // if(consecutive_errors > 0 && consecutive_errors < 3)
        if(consecutive_errors == 1)
            error_is_significant = true;

        // errorlog << "Motor error: " << std::endl;

        // PrintSensorResponseRate(sensor_id);

    } else if(sensor_error_code != SENSOR_ERROR_NONE) {
        std::cout
                // << __PRETTY_FUNCTION__ << ": "
                // << "DS::CFHC()" << ": "
                << "Sensor error: id = '"
                << Robot::SensorReadManager::SensorNameForId(sensor_id)
                << "' ("
                << sensor_id
                << "), error='"
                << GetSensorErrorDescription(sensor_error_code)
                << "';"
                << std::endl;
    } else {
        // PrintSensorResponseRate(sensor_id);
    }

    return error_is_significant;
}

void SensorReadManager::UpdateSensorResponseRates(int error_code)
{
    for (std::vector<SensorReadDescriptor*>::iterator it = descriptor_list_.begin();
         it != descriptor_list_.end(); ++it)
    {
        SensorReadDescriptor* sensor_read = *it;
        sensor_read->UpdateResponseRate(error_code);
    }
}

void SensorReadManager::PrintSensorResponseRates()
{
    for (std::vector<SensorReadDescriptor*>::iterator it = descriptor_heap_.begin();
         it != descriptor_heap_.end(); ++it)
    {
        SensorReadDescriptor* sensor_read = *it;
        PrintSensorResponseRate(sensor_read->sensor_id());
    }
}

void SensorReadManager::PrintSensorResponseRate(int sensor_id)
{
    double response_rate = GetDescriptorById(sensor_id)->response_rate();
    double consecutive_errors = GetDescriptorById(sensor_id)->consecutive_errors();
    std::cout 
        << "  "
        << std::setw(16) << SensorNameForId(sensor_id)
        << " (" << std::setw(3) << sensor_id << "):"
        // << " response_rate = " 
        // << std::setw(6) << response_rate 
        // << ";"
        << " consecutive_errors = " 
        << std::setw(3) << consecutive_errors 
        << ";"
        << std::endl;
}

void SensorReadManager::GetFilteredLikelySensorFailures(
    std::vector<int>* failing_sensors)
{
    static const int arr_sensors_right_arm[] = {
        Robot::JointData::ID_R_SHOULDER_PITCH,
        Robot::JointData::ID_R_SHOULDER_ROLL ,
        Robot::JointData::ID_R_ELBOW         ,
        };
    static const int arr_sensors_left_arm[]  = {
        Robot::JointData::ID_L_SHOULDER_PITCH,
        Robot::JointData::ID_L_SHOULDER_ROLL ,
        Robot::JointData::ID_L_ELBOW         ,
        };
    static const int arr_sensors_right_leg[] = {
        Robot::JointData::ID_R_HIP_YAW       ,
        Robot::JointData::ID_R_HIP_ROLL      ,
        Robot::JointData::ID_R_HIP_PITCH     ,
        Robot::JointData::ID_R_KNEE          ,
        Robot::JointData::ID_R_ANKLE_PITCH   ,
        Robot::JointData::ID_R_ANKLE_ROLL    ,
        Robot::FSR::ID_R_FSR                 ,
        };
    static const int arr_sensors_left_leg[]  = {
        Robot::JointData::ID_L_HIP_YAW       ,
        Robot::JointData::ID_L_HIP_ROLL      ,
        Robot::JointData::ID_L_HIP_PITCH     ,
        Robot::JointData::ID_L_KNEE          ,
        Robot::JointData::ID_L_ANKLE_PITCH   ,
        Robot::JointData::ID_L_ANKLE_ROLL    ,
        Robot::FSR::ID_L_FSR                 ,
        };
    static const int arr_sensors_head[]      = {
        Robot::JointData::ID_HEAD_PAN        ,
        Robot::JointData::ID_HEAD_TILT       ,
        };
    // Note: These vectors should be initialised using initialiser lists
    //       once we start using c++11.
    static std::vector<int> sensors_right_arm(arr_sensors_right_arm, arr_sensors_right_arm + sizeof(arr_sensors_right_arm) / sizeof(arr_sensors_right_arm[0]));
    static std::vector<int> sensors_left_arm (arr_sensors_left_arm , arr_sensors_left_arm  + sizeof(arr_sensors_left_arm ) / sizeof(arr_sensors_left_arm [0]));
    static std::vector<int> sensors_right_leg(arr_sensors_right_leg, arr_sensors_right_leg + sizeof(arr_sensors_right_leg) / sizeof(arr_sensors_right_leg[0]));
    static std::vector<int> sensors_left_leg (arr_sensors_left_leg , arr_sensors_left_leg  + sizeof(arr_sensors_left_leg ) / sizeof(arr_sensors_left_leg [0]));
    static std::vector<int> sensors_head     (arr_sensors_head     , arr_sensors_head      + sizeof(arr_sensors_head     ) / sizeof(arr_sensors_head     [0]));

    // Create a new vector for the results
    *failing_sensors = std::vector<int>();

    SensorReadDescriptor* cm_read = GetDescriptorById(Robot::CM730::ID_CM);
    // double cm_response_rate = cm_read->response_rate();
    double cm_consecutive_errors = cm_read->consecutive_errors();
    // if(0.5 > cm_response_rate)
    if(cm_consecutive_errors > 0)
    {
        failing_sensors->push_back(Robot::CM730::ID_CM);
        // return;
    }

    FilterLimbSensorFailures(sensors_right_arm, *failing_sensors);
    FilterLimbSensorFailures(sensors_left_arm , *failing_sensors);
    FilterLimbSensorFailures(sensors_right_leg, *failing_sensors);
    FilterLimbSensorFailures(sensors_left_leg , *failing_sensors);
    FilterLimbSensorFailures(sensors_head     , *failing_sensors);                
}

bool SensorReadManager::CheckForCM730ResetState()
{
    for (std::vector<SensorReadDescriptor*>::iterator it = descriptor_list_.begin(); 
        it != descriptor_list_.end(); ++it)
    {
        SensorReadDescriptor* sensor_read = *it;

        if(sensor_read->sensor_id() == Robot::CM730::ID_CM)
            continue;

        if(sensor_read->consecutive_errors() == 0)
        {
            std::cout   << "CheckForCM730ResetState sensor: "
                        << sensor_read->sensor_id()
                        << ", errors: " 
                        << sensor_read->consecutive_errors()
                        << std::endl;
            return false;
        }
    }

    return true;
}

void SensorReadManager::FilterLimbSensorFailures(
    std::vector<int>& limb_sensors, 
    std::vector<int>& failing_sensors)
{
    for (std::vector<int>::iterator it = limb_sensors.begin(); 
        it != limb_sensors.end(); ++it)
    {
        int sensor_id = *it;
        SensorReadDescriptor* sensor_read = GetDescriptorById(sensor_id);

        // double response_rate = sensor_read->response_rate();
        int consecutive_errors = sensor_read->consecutive_errors();

        // if(0.5 > response_rate)
        if(consecutive_errors > 0)
        {
            failing_sensors.push_back(sensor_id);

            break;
        }
    }
}

std::string SensorReadManager::GetSensorErrorDescription(unsigned int error_value)
{
    std::string error_description;

    if(error_value == SENSOR_ERROR_NONE) 
        return "No errors";

    if(error_value == -1) 
        return "No response from sensor.";
        // return "All error flags are set.";
    
    if(error_value & SENSOR_ERROR_FLAG_INPUT_VOLTAGE)
        error_description.append("input-voltage error; ");
    
    if(error_value & SENSOR_ERROR_FLAG_ANGLE_LIMIT)
        error_description.append("angle-limit error; ");
    
    if(error_value & SENSOR_ERROR_FLAG_OVERHEATING)
        error_description.append("overheating error; ");
    
    if(error_value & SENSOR_ERROR_FLAG_RANGE)
        error_description.append("range error; ");
    
    if(error_value & SENSOR_ERROR_FLAG_CHECKSUM)
        error_description.append("checksum error; ");
    
    if(error_value & SENSOR_ERROR_FLAG_OVERLOAD)
        error_description.append("overload error; ");
    
    if(error_value & SENSOR_ERROR_FLAG_INSTRUCTION)
        error_description.append("instruction error; ");
    
    return error_description;
}

const char* SensorReadManager::SensorNameForId(int sensor_id)
{
    switch (sensor_id)
    {
    case Robot::JointData::ID_R_SHOULDER_PITCH: return "R_SHOULDER_PITCH";
    case Robot::JointData::ID_L_SHOULDER_PITCH: return "L_SHOULDER_PITCH";
    case Robot::JointData::ID_R_SHOULDER_ROLL : return "R_SHOULDER_ROLL" ;
    case Robot::JointData::ID_L_SHOULDER_ROLL : return "L_SHOULDER_ROLL" ;
    case Robot::JointData::ID_R_ELBOW         : return "R_ELBOW"         ;
    case Robot::JointData::ID_L_ELBOW         : return "L_ELBOW"         ;
    case Robot::JointData::ID_R_HIP_YAW       : return "R_HIP_YAW"       ;
    case Robot::JointData::ID_L_HIP_YAW       : return "L_HIP_YAW"       ;
    case Robot::JointData::ID_R_HIP_ROLL      : return "R_HIP_ROLL"      ;
    case Robot::JointData::ID_L_HIP_ROLL      : return "L_HIP_ROLL"      ;
    case Robot::JointData::ID_R_HIP_PITCH     : return "R_HIP_PITCH"     ;
    case Robot::JointData::ID_L_HIP_PITCH     : return "L_HIP_PITCH"     ;
    case Robot::JointData::ID_R_KNEE          : return "R_KNEE"          ;
    case Robot::JointData::ID_L_KNEE          : return "L_KNEE"          ;
    case Robot::JointData::ID_R_ANKLE_PITCH   : return "R_ANKLE_PITCH"   ;
    case Robot::JointData::ID_L_ANKLE_PITCH   : return "L_ANKLE_PITCH"   ;
    case Robot::JointData::ID_R_ANKLE_ROLL    : return "R_ANKLE_ROLL"    ;
    case Robot::JointData::ID_L_ANKLE_ROLL    : return "L_ANKLE_ROLL"    ;
    case Robot::JointData::ID_HEAD_PAN        : return "HEAD_PAN"        ;
    case Robot::JointData::ID_HEAD_TILT       : return "HEAD_TILT"       ;
    case Robot::FSR::ID_L_FSR                 : return "L_FSR"           ;
    case Robot::FSR::ID_R_FSR                 : return "R_FSR"           ;
    case Robot::CM730::ID_CM                  : return "CM"              ;
    default                                   : return "UNKNOWN_SENSOR"  ;
    }
}
