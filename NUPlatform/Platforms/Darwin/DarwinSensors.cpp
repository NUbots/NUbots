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
    error_fields.resize(NUM_MOTORS);
    for(int i = 0 ; i < NUM_MOTORS ; i++)
    {
        error_fields[i].resize(2);
    }
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

/*! @brief Copys the sensors data from the hardware communication module to the NUSensorsData container
 */
void DarwinSensors::copyFromHardwareCommunications()
{
    int result;
//	if(m_current_time < 2000)
//	{
//		result = cm730->BulkReadLight();
//		return;
//	}


    //Control Board Data:
    copyFromAccelerometerAndGyro();
    copyFromButtons();
    copyFromBattery();

    //Motor Data:
    copyFromJoints();
    copyFromFeet();

    //READ FROM JOINTS and CONTROL BOARD:
    result = cm730->BulkReadLight();

    if(motor_error) {
        #if DEBUG_NUSENSORS_VERBOSITY > 0
            debug << "DarwinSensors::copyFromHardwareCommunications\nMotor error: " <<endl;
        #endif
        errorlog << "Motor error: " <<endl;

        for(int i=0; i<NUM_MOTORS; i++) {
            #if DEBUG_NUSENSORS_VERBOSITY > 0
                debug << "ID: " << error_fields[i][0] << " err: " << error_fields[i][1] << endl;
            #endif
            errorlog << "ID: " << error_fields[i][0] << " err: " << error_fields[i][1] << endl;
        }

//        cm730->DXLPowerOff();
//        platform->msleep(500);
//        cm730->DXLPowerOn();

        motor_error = false;
    }

    //delete CMdatatable;

	
    int num_tries = 0;
    while(result != Robot::CM730::SUCCESS)
    {
//        if(num_tries == 50) {
//            cm730->DXLPowerOff();
//            platform->msleep(500);
//            cm730->DXLPowerOn();
//        }
        #if DEBUG_NUSENSORS_VERBOSITY > 0
            debug << "BulkRead Error: " << result  << " Trying Again " << num_tries <<endl;
        #endif
        errorlog << "BulkRead Error: " << result  << " Trying Again " << num_tries <<endl;
        result = cm730->BulkReadLight();
        //return;
        num_tries++;
    }
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

        //error fields
        error_fields[i][0] = cm730->m_BulkReadData[int(platform->m_servo_IDs[i])].ReadWord(int(Robot::MX28::P_ID));
        error_fields[i][1] = cm730->m_BulkReadData[int(platform->m_servo_IDs[i])].error;
        if(error_fields[i][1] != 0)
            motor_error = true;

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
        joint[NUSensorsData::StiffnessId] = 100*data;
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
    data[0] = cm730->m_BulkReadData[int(Robot::CM730::ID_CM)].ReadWord(addr);
    data[0] = (data[0]-centrevalue)/VALUETORPS_RATIO;
	
    addr = int(Robot::CM730::P_GYRO_Y_L);
    //data[1] = cm730->MakeWord(datatable[addr-start_addr],datatable[addr+1-start_addr]);
    data[1] = cm730->m_BulkReadData[int(Robot::CM730::ID_CM)].ReadWord(addr);
    data[1] = (data[1]-centrevalue)/VALUETORPS_RATIO;

    addr = int(Robot::CM730::P_GYRO_Z_L);
    //data[2] = cm730->MakeWord(datatable[addr-start_addr],datatable[addr+1-start_addr]);
    data[2] = cm730->m_BulkReadData[int(Robot::CM730::ID_CM)].ReadWord(addr);
    data[2] = (data[2]-centrevalue)/VALUETORPS_RATIO;
    //cout << "GYRO: \t(" << data[0] << "," << data[1]<< "," << data[2] << ")"<< endl;
    m_data->set(NUSensorsData::Gyro,m_current_time, data);

    addr = int(Robot::CM730::P_ACCEL_Y_L);
    data[0] = cm730->m_BulkReadData[int(Robot::CM730::ID_CM)].ReadWord(addr);
    //data[0] = cm730->MakeWord(datatable[addr-start_addr],datatable[addr+1-start_addr]);
    data[0] = -(data[0]-centrevalue)/VALUETOACCEL_RATIO;

    addr = int(Robot::CM730::P_ACCEL_X_L);
    data[1] = cm730->m_BulkReadData[int(Robot::CM730::ID_CM)].ReadWord(addr);
    //data[1] = cm730->MakeWord(datatable[addr-start_addr],datatable[addr+1-start_addr]);
    data[1] = (data[1]-centrevalue)/VALUETOACCEL_RATIO;
	
    addr = int(Robot::CM730::P_ACCEL_Z_L);
    data[2] = cm730->m_BulkReadData[int(Robot::CM730::ID_CM)].ReadWord(addr);
    //data[2] = cm730->MakeWord(datatable[addr-start_addr],datatable[addr+1-start_addr]);
    data[2] = -(data[2]-centrevalue)/VALUETOACCEL_RATIO;
	
    m_data->set(NUSensorsData::Accelerometer,m_current_time, data);
}

void DarwinSensors::copyFromFeet()
{
    //Darwin has no feet sensors atm.
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
    //Values are 10x higher then actual present voltage.
	
    int addr = Robot::CM730::P_VOLTAGE;
    int data  = cm730->m_BulkReadData[int(Robot::CM730::ID_CM)].ReadWord(addr);
    float battery_percentage = data/120.00 *100.00;
    m_data->set(NUSensorsData::BatteryVoltage, m_current_time, battery_percentage); //Convert to percent
    return;
}
