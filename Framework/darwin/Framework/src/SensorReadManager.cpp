#include "FSR.h"
#include "CM730.h"
#include "MotionStatus.h"
#include "SensorReadManager.h"
#include "SensorReadDescriptor.h"

using namespace Robot;

// Provide definition of constants here to prevent linker error.
// (am not sure why this is necessary)
const int Robot::FSR::ID_R_FSR;
const int Robot::FSR::ID_L_FSR;

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


//! Filters the list of response rates to return a list of sensor IDs that
//! Are actually failing.
//! (i.e. Removes likely false-positives from the list of failing sensors.
//! Currently assumes that all sensors are always in use.
//! Note: If three sensors on the same limb are not responding, it is likely 
//!       that only the first of them requires attention)
void SensorReadManager::GetFilteredLikelySensorFailures(
    std::vector<int>* failing_sensors)
{
    static std::vector<int> sensors_right_arm;// {
    //     Robot::JointData::ID_R_SHOULDER_PITCH,
    //     Robot::JointData::ID_R_SHOULDER_ROLL ,
    //     Robot::JointData::ID_R_ELBOW         ,
    //     };
    // static std::vector<int> sensors_left_arm  {
    //     Robot::JointData::ID_L_SHOULDER_PITCH,
    //     Robot::JointData::ID_L_SHOULDER_ROLL ,
    //     Robot::JointData::ID_L_ELBOW         ,
    //     };
    // static std::vector<int> sensors_right_leg {
    //     Robot::JointData::ID_R_HIP_YAW       ,
    //     Robot::JointData::ID_R_HIP_ROLL      ,
    //     Robot::JointData::ID_R_HIP_PITCH     ,
    //     Robot::JointData::ID_R_KNEE          ,
    //     Robot::JointData::ID_R_ANKLE_PIT     ,
    //     Robot::JointData::ID_R_ANKLE_ROL     ,
    //     Robot::FSR::ID_R_FSR                 ,
    //     };
    // static std::vector<int> sensors_left_leg  {
    //     Robot::JointData::ID_L_HIP_YAW       ,
    //     Robot::JointData::ID_L_HIP_ROLL      ,
    //     Robot::JointData::ID_L_HIP_PITCH     ,
    //     Robot::JointData::ID_L_KNEE          ,
    //     Robot::JointData::ID_L_ANKLE_PITCH   ,
    //     Robot::JointData::ID_L_ANKLE_ROLL    ,
    //     Robot::FSR::ID_L_FSR                 ,
    //     };
    // static std::vector<int> sensors_head      {
    //     Robot::JointData::ID_HEAD_PAN        ,
    //     Robot::JointData::ID_HEAD_TILT       ,
    //     };

    // Create a new vector for the results
    *failing_sensors = std::vector<int>();

    // Assume that a CM failure prevents all sensors from working
    // (should test this, + remove this comment (or amend the code) soon)
    double cm_response_rate = descriptor_map[Robot::CM730::ID_CM]->response_rate();
    if(0.5 > cm_response_rate)
    {
        failing_sensors->push_back(Robot::CM730::ID_CM);
        return;
    }

    FilterLimbSensorFailures(sensors_right_arm, *failing_sensors);
    // FilterLimbSensorFailures(sensors_left_leg , *failing_sensors);
    // FilterLimbSensorFailures(sensors_right_arm, *failing_sensors);
    // FilterLimbSensorFailures(sensors_left_leg , *failing_sensors);
    // FilterLimbSensorFailures(sensors_head     , *failing_sensors);                
}

void SensorReadManager::FilterLimbSensorFailures(
    std::vector<int>& limb_sensors, 
    std::vector<int>& failing_sensors)
{
    for (std::vector<int>::iterator it = limb_sensors.begin(); 
        it != limb_sensors.end(); ++it)
    {
        int sensor_id = *it;
        double response_rate = descriptor_map[sensor_id]->response_rate();
        if(0.5 > response_rate)
        {
            failing_sensors.push_back(sensor_id);

            break;
        }
    }
}

void SensorReadManager::Initialize()
{
    // Should consider pinging things sometime:
    // Ping(CM730::ID_CM, NULL) == SUCCESS

    // Create the read descriptors.
    descriptor_list.clear();

    // CM730:
    SensorReadDescriptor cm_read;
    cm_read.set_sensor_id(CM730::ID_CM);
    cm_read.set_start_address(CM730::P_BUTTON);
    cm_read.set_num_bytes(20);
    descriptor_list.push_back(cm_read);
    descriptor_heap.push_back(&cm_read);
    descriptor_map[CM730::ID_CM] = &cm_read;

    // Servo motors:
    for(int servo_id = 1; servo_id < JointData::NUMBER_OF_JOINTS; ++servo_id)
    {
        SensorReadDescriptor servo_read;
        servo_read.set_sensor_id(servo_id);
        servo_read.set_start_address(MX28::P_PRESENT_POSITION_L);
        servo_read.set_num_bytes(2);
        descriptor_list.push_back(servo_read);
        descriptor_heap.push_back(&servo_read);
        descriptor_map[servo_id] = &servo_read;
    }

    // Force-sensitive resistors:
    SensorReadDescriptor fsr_l_read;
    fsr_l_read.set_sensor_id(FSR::ID_L_FSR);
    fsr_l_read.set_start_address(FSR::P_FSR1_L);
    fsr_l_read.set_num_bytes(10);
    descriptor_list.push_back(fsr_l_read);
    descriptor_heap.push_back(&fsr_l_read);
    descriptor_map[FSR::ID_L_FSR] = &fsr_l_read;

    SensorReadDescriptor fsr_r_read;
    fsr_r_read.set_sensor_id(Robot::FSR::ID_R_FSR);
    fsr_r_read.set_start_address(FSR::P_FSR1_L);
    fsr_r_read.set_num_bytes(10);
    descriptor_list.push_back(fsr_r_read);
    descriptor_heap.push_back(&fsr_r_read);
    descriptor_map[Robot::FSR::ID_R_FSR] = &fsr_r_read;

    // // Make the initial bulk read packet.
    // MakeBulkReadPacket();
}

void SensorReadManager::MakeBulkReadPacket(unsigned char* bulk_read_tx_packet_)
{
    const int kDataStart = (PARAMETER) + 1;

    std::make_heap(
        descriptor_heap.begin(), 
        descriptor_heap.end(),
        CompareSensorReadDescriptors());

    // Make Data Packet
    int pos = 0;
    // for (std::vector<SensorReadDescriptor>::iterator it = descriptor_heap.begin();
    //      it != descriptor_heap.end(); 
    //      ++it)
    for(int i = 0; i < (int)descriptor_heap.size(); i++)
    {
        SensorReadDescriptor* sensor_read = descriptor_heap.front();
        std::pop_heap(
            descriptor_heap.begin(), 
            descriptor_heap.end(),
            CompareSensorReadDescriptors());

        bulk_read_tx_packet_[kDataStart + pos++] = sensor_read->num_bytes();
        bulk_read_tx_packet_[kDataStart + pos++] = sensor_read->sensor_id();
        bulk_read_tx_packet_[kDataStart + pos++] = sensor_read->start_address();
    }

    // Make Packet Header
    bulk_read_tx_packet_[ID]          = (unsigned char)CM730::ID_BROADCAST;
    bulk_read_tx_packet_[LENGTH]      = pos + 3;
    bulk_read_tx_packet_[INSTRUCTION] = INST_BULK_READ;
    bulk_read_tx_packet_[PARAMETER]   = (unsigned char)0x0;
    // descriptor_heap.length() * 3 + 3
}

SensorReadDescriptor& SensorReadManager::operator[](const int index)
{
    return *(descriptor_map[index]);
}

// // Initialise response rates:
// void SensorReadManager::InitialiseSensorResponseRates()
// {
//     sensor_response_rates[112] = 1.0; // Robot::FSR::ID_L_FSR
//     sensor_response_rates[111] = 1.0; // Robot::FSR::ID_R_FSR
//     sensor_response_rates[Robot::CM730::ID_CM ] = 1.0;
//     for (std::vector<int>::iterator it = platform->m_servo_IDs.begin();
//         it != platform->m_servo_IDs.end(); ++it)
//     {
//        int servo_id = *it;
//        sensor_response_rates[servo_id] = 1.0;
//     }
// }

void SensorReadManager::UpdateSensorResponseRates(int error_code)
{
    for (std::vector<SensorReadDescriptor>::iterator it = descriptor_list.begin();
         it != descriptor_list.end(); ++it)
    {
        SensorReadDescriptor& sensor_read = *it;
        sensor_read.UpdateResponseRate(error_code);
    }
}

void SensorReadManager::PrintSensorResponseRates()
{
    for (std::vector<SensorReadDescriptor*>::iterator it = descriptor_heap.begin();
         it != descriptor_heap.end(); ++it)
    {
        SensorReadDescriptor* sensor_read = *it;
        PrintSensorResponseRate(sensor_read->sensor_id());
    }
}

void SensorReadManager::PrintSensorResponseRate(int sensor_id)
{
    double response_rate = descriptor_map[sensor_id]->response_rate();
    std::cout 
        << SensorNameForId(sensor_id)
        << " response_rate = " 
        << response_rate 
        << ";"
        << std::endl;
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