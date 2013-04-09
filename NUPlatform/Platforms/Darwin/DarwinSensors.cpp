/*! @file DarwinSensors.cpp
    @brief Implementation of Darwin sensor class

    @author Jason Kulk
 
  Copyright (c) 2010 Jason Kulk
 
 This file is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This file is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <limits>


#include "DarwinSensors.h"
#include "DarwinPlatform.h"
#include "DarwinJointMapping.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Tools/Math/General.h"

#include "debug.h"
#include "debugverbositynusensors.h"

#include "Framework/darwin/Framework/include/CM730.h"
#include "Framework/darwin/Framework/include/JointData.h"

// Error flags returned by sensor + servo reads/commands.
#define SENSOR_ERROR_NONE                  (0x0000)
#define SENSOR_ERROR_FLAG_INPUT_VOLTAGE    (0x0001)
#define SENSOR_ERROR_FLAG_ANGLE_LIMIT      (0x0002)
#define SENSOR_ERROR_FLAG_OVERHEATING      (0x0004)
#define SENSOR_ERROR_FLAG_RANGE            (0x0008)
#define SENSOR_ERROR_FLAG_CHECKSUM         (0x0010)
#define SENSOR_ERROR_FLAG_OVERLOAD         (0x0020)
#define SENSOR_ERROR_FLAG_INSTRUCTION      (0x0040)

// Note: These defines are simply copied from those in CM730.cpp
#define ID					(2)
#define LENGTH				(3)
#define INSTRUCTION			(4)
#define ERRBIT				(4)
#define PARAMETER			(5)
#define DEFAULT_BAUDNUMBER	(1)

#define INST_PING			(1)
#define INST_READ			(2)
#define INST_WRITE			(3)
#define INST_REG_WRITE		(4)
#define INST_ACTION			(5)
#define INST_RESET			(6)
#define INST_SYNC_WRITE		(131)   // 0x83
#define INST_BULK_READ      (146)   // 0x92

using namespace std;

/*! @brief Constructs a nubot sensor class with Darwin backend
 */
DarwinSensors::DarwinSensors(DarwinPlatform* darwin, Robot::CM730* subboard)
{
    #if DEBUG_NUSENSORS_VERBOSITY > 0
        debug << "DarwinSensors::DarwinSensors()" << endl;
    #endif

    platform = darwin;
    cm730 = subboard;

    m_data->addSensors(platform->m_servo_names);

    m_joint_ids = m_data->mapIdToIds(NUSensorsData::All);
    m_previous_positions = vector<float>(platform->m_servo_names.size(), 0);
    m_previous_velocities = vector<float>(platform->m_servo_names.size(), 0);
    m_joint_mapping = &DarwinJointMapping::Instance();

    std::vector<float> invalid(NUSensorsData::NumEndEffectorIndices, numeric_limits<float>::quiet_NaN());
    m_data->set(NUSensorsData::RLegEndEffector, m_data->CurrentTime, invalid);
    m_data->set(NUSensorsData::LLegEndEffector, m_data->CurrentTime, invalid);

    motor_error = false;
}

/*! @brief Destructor for DarwinSensors
 */
DarwinSensors::~DarwinSensors()
{
    #if DEBUG_NUSENSORS_VERBOSITY > 0
        debug << "DarwinSensors::~DarwinSensors()" << endl;
    #endif
    delete cm730;
}

std::string DarwinSensors::getSensorErrorDescription(unsigned int error_value)
{
    std::string error_description;

    if(error_value == SENSOR_ERROR_NONE) 
        return "No Errors";

    if(error_value == -1) 
        return "All error flags are set.";
    
    if(error_value & SENSOR_ERROR_FLAG_INPUT_VOLTAGE)
        error_description.append("Input Voltage Error; ");
    
    if(error_value & SENSOR_ERROR_FLAG_ANGLE_LIMIT)
        error_description.append("Angle Limit Error; ");
    
    if(error_value & SENSOR_ERROR_FLAG_OVERHEATING)
        error_description.append("OverHeating Error; ");
    
    if(error_value & SENSOR_ERROR_FLAG_RANGE)
        error_description.append("Range Error; ");
    
    if(error_value & SENSOR_ERROR_FLAG_CHECKSUM)
        error_description.append("Checksum Error; ");
    
    if(error_value & SENSOR_ERROR_FLAG_OVERLOAD)
        error_description.append("Overload Error; ");
    
    if(error_value & SENSOR_ERROR_FLAG_INSTRUCTION)
        error_description.append("Instruction Error; ");
    
    return error_description;
}

/*! @brief Copys the sensors data from the hardware communication module to the NUSensorsData container
 */
void DarwinSensors::copyFromHardwareCommunications()
{
    // 1. Copy data from last bulk read of the CM730.

    //Control Board Data:
    copyFromAccelerometerAndGyro();
    copyFromButtons();
    copyFromBattery();

    //Motor Data:
    copyFromJoints();
    copyFromFeet();

    // 2. Read data in bulk from the CM730 controller board 
    //    (i.e. read all sensor and motor data for the next iteration)
//         #if DEBUG_NUSENSORS_VERBOSITY > 0
//         debug << "Motor error: " << endl;
//         #endif
//         errorlog << "Motor error: " << endl;
// //        cm730->DXLPowerOff();
// //        platform->msleep(500);
// //        cm730->DXLPowerOn();

    bool repeat_bulk_read = false;
    do
    {
        // Perform the read operation from the CM730.
        // Note: Possible error codes are:
        //   - SUCCESS
        //   - TX_CORRUPT
        //   - TX_FAIL
        //   - RX_FAIL
        //   - RX_TIMEOUT
        //   - RX_CORRUPT
        int bulk_read_error_code = cm730->BulkRead();
        
        if(bulk_read_error_code != Robot::CM730::SUCCESS)
        {
            cout   << __PRETTY_FUNCTION__ << ": "
                    << "BULK READ ERROR: "
                    << Robot::CM730::getTxRxErrorString(bulk_read_error_code)
                    << endl;

            // Check for servo read errors: (and also other sensors)
            // Note: Possible error flags are:
            //   - SENSOR_ERROR_NONE
            //   - SENSOR_ERROR_FLAG_INPUT_VOLTAGE
            //   - SENSOR_ERROR_FLAG_ANGLE_LIMIT
            //   - SENSOR_ERROR_FLAG_OVERHEATING
            //   - SENSOR_ERROR_FLAG_RANGE
            //   - SENSOR_ERROR_FLAG_CHECKSUM
            //   - SENSOR_ERROR_FLAG_OVERLOAD
            //   - SENSOR_ERROR_FLAG_INSTRUCTION
            bool servo_read_error = false;
            for (size_t i = 0; i < platform->m_servo_IDs.size(); i++)
            {
                int servo_ID   = int(platform->m_servo_IDs[i]);
                int servo_error_code = cm730->m_BulkReadData[servo_ID].error;

                if(servo_error_code != SENSOR_ERROR_NONE)
                {
                    // keep track of which sensors failed?
                    // (use a list - speed doesn't matter much, since it 
                    // shouldn't be used often if the robot is functioning correctly)
                    servo_read_error = true;
                    //         errorlog << "Motor error: " << endl;
                    cout   << __PRETTY_FUNCTION__ << ": "
                            << "Motor error: id = '"
                            << Robot::JointData::GetJointName(servo_ID) 
                            << "', error='"
                            << getSensorErrorDescription(servo_error_code)
                            << "';"
                            << endl;
                }
            }
            cout << endl;

            // Decide whether to repeat the read based on errors returned:
            repeat_bulk_read = true;

            // // cycle dynamixel power?
        }
    } while (repeat_bulk_read);

    return;
}

/*! @brief Copys the joint sensor data 
 */
void DarwinSensors::copyFromJoints()
{
    static const float NaN = numeric_limits<float>::quiet_NaN();
    vector<float> joint(NUSensorsData::NumJointSensorIndices, NaN);
    float delta_t = (m_current_time - m_previous_time)/1000;
    int data;
    int addr;
	
    //int start_addr = int(Robot::MX28::P_TORQUE_ENABLE);
    //int end_addr   = int(Robot::MX28::P_PRESENT_TEMPERATURE);

    //int start_addr = 0;
    //int table_start_addr = 0;
    //int end_addr   = int(Robot::MX28::P_PRESENT_TEMPERATURE);
    //int datasize   = end_addr-start_addr+1;
    //unsigned char* datatable = new unsigned char[datasize+1];
    //int addr;
    //int error;
    motor_error = false;

    for (size_t i=0; i < platform->m_servo_IDs.size(); i++)
    {
//        int result = cm730->ReadTable(int(platform->m_servo_IDs[i]),table_start_addr,end_addr,datatable,&error);
//        if(result != Robot::CM730::SUCCESS)
//        {
//            debug << "Sensor " << platform->m_servo_IDs[i] <<  " failed."<< endl;
//            continue;
//        }

        addr = int(Robot::MX28::P_PRESENT_POSITION_L);
        data = cm730->m_BulkReadData[int(platform->m_servo_IDs[i])].ReadWord(addr);

        //cm730->MakeWord(datatable[addr-start_addr],datatable[addr-start_addr+1]);

        joint[NUSensorsData::PositionId] = m_joint_mapping->raw2joint(i, data);

        /*
        // Extra values - Not currently used.
        addr = int(Robot::MX28::P_GOAL_POSITION_L);
        data = cm730->m_BulkReadData[int(platform->m_servo_IDs[i])].ReadWord(addr);
        //data = cm730->MakeWord(datatable[addr-start_addr],datatable[addr+1-start_addr]);
        joint[NUSensorsData::TargetId] = Value2Radian(data) + platform->m_servo_Offsets[i];
		
        addr = int(Robot::MX28::P_MOVING_SPEED_L);
        data = cm730->m_BulkReadData[int(platform->m_servo_IDs[i])].ReadWord(addr);
        //data = cm730->MakeWord(datatable[addr-start_addr],datatable[addr+1-start_addr]);
        joint[NUSensorsData::VelocityId] = data;

        addr = int(Robot::MX28::P_PRESENT_TEMPERATURE);
        data = cm730->m_BulkReadData[int(platform->m_servo_IDs[i])].ReadByte(addr);
        //data = int(datatable[addr-start_addr]);
        joint[NUSensorsData::TemperatureId] = data;

        addr = int(Robot::MX28::P_TORQUE_ENABLE);
        data = cm730->m_BulkReadData[int(platform->m_servo_IDs[i])].ReadByte(addr);
        //data = int(datatable[addr-start_addr]);
        joint[NUSensorsData::StiffnessId] = 100*data; // 'NUSensorsData::StiffnessId' can't be right? It's used for the actual 'stiffness' value.
		*/
        addr = int(Robot::MX28::P_PRESENT_LOAD_L);
        data = (int)cm730->m_BulkReadData[int(platform->m_servo_IDs[i])].ReadWord(addr);
        //data = cm730->MakeWord(datatable[addr-start_addr],datatable[addr+1-start_addr]);
        joint[NUSensorsData::TorqueId] = data*1.262e-3;
        //<! Current is blank
        //joint[NUSensorsData::AccelerationId] = (joint[NUSensorsData::VelocityId] - m_previous_velocities[i])/delta_t;
        //<! Copy into m_data
        

        //Calculate Speed:
        joint[NUSensorsData::VelocityId] = (joint[NUSensorsData::PositionId] - m_previous_positions[i])/delta_t;
        //Calculate Acceleration:
        joint[NUSensorsData::AccelerationId] = (joint[NUSensorsData::VelocityId] - m_previous_velocities[i])/delta_t;
        //Get Local Goal Position:
        joint[NUSensorsData::TargetId] = platform->getMotorGoalPosition(i);
        //Get Local Stiffness:
        joint[NUSensorsData::StiffnessId] = platform->getMotorStiffness(i);

        m_data->set(*m_joint_ids[i], m_current_time, joint);

        // Update historic variables.
        m_previous_positions[i] = joint[NUSensorsData::PositionId];
        m_previous_velocities[i] = joint[NUSensorsData::VelocityId];

        #if DEBUG_NUSENSORS_VERBOSITY > 0
            debug << "DarwinSensors::CopyFromJoints " << i << " " << joint[NUSensorsData::PositionId]<<endl;
    	#endif
    }
}

void DarwinSensors::copyFromAccelerometerAndGyro()
{
    //<! Get the data from the control board:
    //int start_addr = 0;
    //int end_addr   = int(Robot::CM730::P_VOLTAGE);
    //int datasize   = end_addr-start_addr+1;
    //unsigned char* datatable = new unsigned char[datasize];
    //int error = 0;
    //cm730->ReadTable(start_addr,end_addr,datatable,&error);
    float VALUETORPS_RATIO = 18.3348;//512/27.925
    float VALUETOACCEL_RATIO = 0.1304; //512/4*981
    int addr;
    int x,y,z;
    float centrevalue = 512;
    vector<float> data(3,0);


    //<! Assign the robot data to the NUSensor Structure:
    addr = int(Robot::CM730::P_GYRO_X_L);
    //data[0] = cm730->MakeWord(datatable[addr-start_addr],datatable[addr+1-start_addr]);
    float tGx = data[0] = cm730->m_BulkReadData[int(Robot::CM730::ID_CM)].ReadWord(addr);
    data[0] = (data[0]-centrevalue)/VALUETORPS_RATIO;
	
    addr = int(Robot::CM730::P_GYRO_Y_L);
    //data[1] = cm730->MakeWord(datatable[addr-start_addr],datatable[addr+1-start_addr]);
    float tGy = data[1] = cm730->m_BulkReadData[int(Robot::CM730::ID_CM)].ReadWord(addr);
    data[1] = (data[1]-centrevalue)/VALUETORPS_RATIO;

    addr = int(Robot::CM730::P_GYRO_Z_L);
    //data[2] = cm730->MakeWord(datatable[addr-start_addr],datatable[addr+1-start_addr]);
    float tGz = data[2] = cm730->m_BulkReadData[int(Robot::CM730::ID_CM)].ReadWord(addr);
    data[2] = (data[2]-centrevalue)/VALUETORPS_RATIO;

   // cout << "GYRO: \t(" << data[0] << "," << data[1]<< "," << data[2] << ")"<< endl;
    //cout << "GYRO_RAW: \t(" << tGx << "," << tGy << "," << tGz << ")"<< endl;

    m_data->set(NUSensorsData::Gyro,m_current_time, data);

    addr = int(Robot::CM730::P_ACCEL_Y_L);
    float tAx = data[0] = cm730->m_BulkReadData[int(Robot::CM730::ID_CM)].ReadWord(addr);
    //data[0] = cm730->MakeWord(datatable[addr-start_addr],datatable[addr+1-start_addr]);
    data[0] = -(data[0]-centrevalue)/VALUETOACCEL_RATIO;

    addr = int(Robot::CM730::P_ACCEL_X_L);
    float tAy = data[1] = cm730->m_BulkReadData[int(Robot::CM730::ID_CM)].ReadWord(addr);
    //data[1] = cm730->MakeWord(datatable[addr-start_addr],datatable[addr+1-start_addr]);
    data[1] = (data[1]-centrevalue)/VALUETOACCEL_RATIO;
	
    addr = int(Robot::CM730::P_ACCEL_Z_L);
    float tAz = data[2] = cm730->m_BulkReadData[int(Robot::CM730::ID_CM)].ReadWord(addr);
    //data[2] = cm730->MakeWord(datatable[addr-start_addr],datatable[addr+1-start_addr]);
    data[2] = -(data[2]-centrevalue)/VALUETOACCEL_RATIO;
	
    //cout << "ACCEL: \t(" << data[0] << "," << data[1]<< "," << data[2] << ")"<< endl;
  //  cout << "ACCEL_RAW: \t(" << tAx << "," << tAy << "," << tAz << ")"<< endl;

    m_data->set(NUSensorsData::Accelerometer,m_current_time, data);
}

void DarwinSensors::copyFromFeet()
{
    int fsr1 = int(Robot::FSR::P_FSR1_L);
    int fsr2 = int(Robot::FSR::P_FSR2_L);
    int fsr3 = int(Robot::FSR::P_FSR3_L);
    int fsr4 = int(Robot::FSR::P_FSR4_L);

    std::vector<float> right_fsr(4,0.f);
    std::vector<float> left_fsr(4,0.f);

    // Darwin FSR give value in milli newtons - we want newtons
    const float fsr_scale_factor = 1e-3;

    int right_fsr_error = cm730->m_BulkReadData[int(Robot::FSR::ID_R_FSR)].error;
    int left_fsr_error = cm730->m_BulkReadData[int(Robot::FSR::ID_L_FSR)].error;

    // Test if the FSR sensors are available.
    if(right_fsr_error == 0 and left_fsr_error == 0)
    {
        // For NUbot system FSR should be in order:
        // front left
        // front right
        // back right
        // back left

        // For right foot, fsr positions are as follows:
        // 1 - front left
        // 2 - front right
        // 3 - back right
        // 4 - back left

        right_fsr[0] = fsr_scale_factor * cm730->m_BulkReadData[int(Robot::FSR::ID_R_FSR)].ReadWord(fsr1);
        right_fsr[1] = fsr_scale_factor * cm730->m_BulkReadData[int(Robot::FSR::ID_R_FSR)].ReadWord(fsr2);
        right_fsr[2] = fsr_scale_factor * cm730->m_BulkReadData[int(Robot::FSR::ID_R_FSR)].ReadWord(fsr3);
        right_fsr[3] = fsr_scale_factor * cm730->m_BulkReadData[int(Robot::FSR::ID_R_FSR)].ReadWord(fsr4);

        // For left foot, fsr positions are as follows:
        // 1 - back right
        // 2 - back left
        // 3 - front left
        // 4 - front right

        left_fsr[0] = fsr_scale_factor * cm730->m_BulkReadData[int(Robot::FSR::ID_L_FSR)].ReadWord(fsr3);
        left_fsr[1] = fsr_scale_factor * cm730->m_BulkReadData[int(Robot::FSR::ID_L_FSR)].ReadWord(fsr4);
        left_fsr[2] = fsr_scale_factor * cm730->m_BulkReadData[int(Robot::FSR::ID_L_FSR)].ReadWord(fsr1);
        left_fsr[3] = fsr_scale_factor * cm730->m_BulkReadData[int(Robot::FSR::ID_L_FSR)].ReadWord(fsr2);

        // Write to sensor values.
        m_data->set(NUSensorsData::RFootTouch, m_current_time, right_fsr);
        m_data->set(NUSensorsData::LFootTouch, m_current_time, left_fsr);
    }
    else
    {
        // Invalidate if values could not be found.
        m_data->setAsInvalid(NUSensorsData::RFootTouch);
        m_data->setAsInvalid(NUSensorsData::LFootTouch);
    }
    return;
}

void DarwinSensors::copyFromButtons()
{
    //Bit 0 <= Mode Button
    //Bit 1 <= Start Button
 
    int addr = Robot::CM730::P_BUTTON;
    int data  = cm730->m_BulkReadData[int(Robot::CM730::ID_CM)].ReadByte(addr);

    if(data == 1)
    {
        //Mode Button Pressed:
        m_data->modify(NUSensorsData::LeftButton, NUSensorsData::StateId, m_current_time, 1);
        //cout << "Mode Button Pressed" << endl;
    }
    else
    {
        m_data->modify(NUSensorsData::LeftButton, NUSensorsData::StateId, m_current_time, 0);
        //m_data->modify(NUSensorsData::MainButton, NUSensorsData::StateId, m_current_time, 0);
    }
    if (data == 2)
    {
        //Start Button Pressed:
        m_data->modify(NUSensorsData::MainButton, NUSensorsData::StateId, m_current_time, 1);
        //cout << "Start Button Pressed" << endl;
    }
    else
    {
        //m_data->modify(NUSensorsData::LeftButton, NUSensorsData::StateId, m_current_time, 0);
        m_data->modify(NUSensorsData::MainButton, NUSensorsData::StateId, m_current_time, 0);
    }

    if (data == 3)
    {
        //Mode and Start Button Pressed:
        //m_data->modify(NUSensorsData::LeftButton, NUSensorsData::StateId, m_current_time, 1);
        //m_data->modify(NUSensorsData::MainButton, NUSensorsData::StateId, m_current_time, 1);
        //cout << "Mode and Start Button Pressed" << endl;
        m_data->modify(NUSensorsData::RightButton, NUSensorsData::StateId, m_current_time, 1);
    }
    else
    {
        m_data->modify(NUSensorsData::RightButton, NUSensorsData::StateId, m_current_time, 0);
    }
    return;
}

void DarwinSensors::copyFromBattery()
{
    //External Voltage is 8-15V
    //Values are 10x higher than actual present voltage.
	
    int addr = Robot::CM730::P_VOLTAGE;
    int data  = cm730->m_BulkReadData[int(Robot::CM730::ID_CM)].ReadWord(addr);
    float battery_percentage = data/120.00 *100.00;
    m_data->set(NUSensorsData::BatteryVoltage, m_current_time, battery_percentage); //Convert to percent
    return;
}
