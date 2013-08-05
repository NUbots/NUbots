#include "Darwin.h"

#include <thread>
#include <algorithm>

// Initialize all of the sensor handler objects using the passed uart
Darwin::Darwin::Darwin(const char* name) :
m_uart(name),
m_cm730(m_uart, ID::CM730),
m_rShoulderPitch(m_uart, ID::R_SHOULDER_PITCH),
m_lShoulderPitch(m_uart, ID::L_SHOULDER_PITCH),
m_rShoulderRoll(m_uart, ID::R_SHOULDER_ROLL),
m_lShoulderRoll(m_uart, ID::L_SHOULDER_ROLL),
m_rElbow(m_uart, ID::R_ELBOW),
m_lElbow(m_uart, ID::L_ELBOW),
m_rHipYaw(m_uart, ID::R_HIP_YAW),
m_lHipYaw(m_uart, ID::L_HIP_YAW),
m_rHipRoll(m_uart, ID::R_HIP_ROLL),
m_lHipRoll(m_uart, ID::L_HIP_ROLL),
m_rHipPitch(m_uart, ID::R_HIP_PITCH),
m_lHipPitch(m_uart, ID::L_HIP_PITCH),
m_rKnee(m_uart, ID::R_KNEE),
m_lKnee(m_uart, ID::L_KNEE),
m_rAnklePitch(m_uart, ID::R_ANKLE_PITCH),
m_lAnklePitch(m_uart, ID::L_ANKLE_PITCH),
m_rAnkleRoll(m_uart, ID::R_ANKLE_ROLL),
m_lAnkleRoll(m_uart, ID::L_ANKLE_ROLL),
m_headPan(m_uart, ID::HEAD_PAN),
m_headTilt(m_uart, ID::HEAD_TILT),
m_rFSR(m_uart, ID::R_FSR),
m_lFSR(m_uart, ID::L_FSR) {
    
    // Turn on the dynamixel power
    m_cm730.turnOnDynamixel();
    
    // Wait about 300ms for the dynamixels to start up
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    
    // Build our bulk read packet
    buildBulkReadPacket();
}

std::vector<std::pair<uint8_t, bool>> Darwin::Darwin::selfTest() {
    
    std::vector<std::pair<uint8_t, bool>> results;
    
    // Ping our CM730
    results.push_back(std::make_pair(ID::CM730, m_cm730.ping()));
    
    // Ping all our motors
    for (int i = 0; i < 20; ++i) {
        results.push_back(std::make_pair(i + 1, (&m_rShoulderPitch)[i].ping()));
    }
    
    // Ping our two FSRs
    results.push_back(std::make_pair(ID::L_FSR, m_lFSR.ping()));
    results.push_back(std::make_pair(ID::R_FSR, m_rFSR.ping()));
    
    return results;
}

void Darwin::Darwin::buildBulkReadPacket() {
    
    // Double check that our type is big enough to hold the result
    static_assert(sizeof(Types::CM730Data) == CM730::Address::VOLTAGE - CM730::Address::LED_PANNEL + 1,
                  "The CM730 type is the wrong size");
    
    // Double check that our type is big enough to hold the result
    static_assert(sizeof(Types::MX28Data) == MX28::Address::PRESENT_TEMPERATURE - MX28::Address::TORQUE_ENABLE + 1,
                  "The MX28 type is the wrong size");
    
    // Double check that our type is big enough to hold the result
    static_assert(sizeof(Types::FSRData) == FSR::Address::FSR_Y - FSR::Address::FSR1_L + 1,
                  "The FSR type is the wrong size");
    
    
    
    // Do a self test so that we can move all the sensors that are currently failing to the end of the list
    std::vector<std::pair<uint8_t, bool>> sensors = selfTest();
    std::sort(std::begin(sensors), std::end(sensors), [](const std::pair<uint8_t, bool>& a, const std::pair<uint8_t, bool>& b) {
        return a.second > b.second;
    });
    
    // This holds our request paramters
    std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> request;
    
    // We run through our sorted list, this way any failing sensors are put at the back of the list
    for (const auto& sensor : sensors) {
        switch (sensor.first) {
                
            // If it's the CM730
            case ID::CM730:
                request.push_back(std::make_tuple(CM730::Address::LED_PANNEL, ID::CM730, sizeof(Types::CM730Data)));
                break;
                
            // If it's the FSRs
            case ID::L_FSR:
            case ID::R_FSR:
                request.push_back(std::make_tuple(FSR::Address::FSR1_L, sensor.first, sizeof(Types::FSRData)));
                break;
            
            // Otherwise we assume that it's a motor
            default:
                request.push_back(std::make_tuple(MX28::Address::TORQUE_ENABLE, sensor.first, sizeof(Types::MX28Data)));
                break;
        }
    }
    
    // Our actual packet we are sending
    std::vector<uint8_t> command(7 + (request.size() * 3));
    command[Packet::MAGIC] = 0xFF;
    command[Packet::MAGIC + 1] = 0xFF;
    command[Packet::ID] = ID::BROADCAST;
    command[Packet::LENGTH] = 3 + (request.size() * 3);
    command[Packet::INSTRUCTION] = DarwinDevice::Instruction::BULK_READ;
    command[Packet::PARAMETER] = 0x00;
    
    // Copy our parameters in
    memcpy(&command[Packet::PARAMETER + 1], request.data(), request.size() * 3);
    
    // Calculate our checksum
    command.back() = calculateChecksum(command.data());
    
    // Swap our command into the actual command location
    m_bulkReadCommand.swap(command);
}

Darwin::BulkReadResults Darwin::Darwin::bulkRead() {
    
    // Execute the BulkRead command
    std::vector<CommandResult> results = m_uart.executeBulk(m_bulkReadCommand);
    
    BulkReadResults data;
    
    bool firstError = true;
    
    for(size_t i = 0; i < results.size(); ++i) {
        
        auto& r = results[i];
        if(r.header.errorcode == ErrorCode::NONE) {
            
            // Copy for motor data
            if(r.header.id >= ID::R_SHOULDER_PITCH && r.header.id <= ID::HEAD_TILT)
                memcpy(&data.motors[r.header.id - 1], r.data.data(), sizeof(Types::MX28Data));
            
            // Copy for FSR data
            else if(r.header.id == ID::R_FSR || r.header.id == ID::L_FSR)
                memcpy(data.fsr + (r.header.id - ID::R_FSR), r.data.data(), sizeof(Types::FSRData));
            
            // Copy CM730 data
            else if(r.header.id == ID::CM730)
                memcpy(&data, r.data.data(), sizeof(Types::CM730Data));
        }
        
        // If it is the first error we have encountered, and it is not the end of the list
        else if(firstError && i != results.size() - 1) {
            
            uint8_t bytes[3];
            bytes[0] = m_bulkReadCommand[Packet::PARAMETER + 1 + (i * 3) + 0];
            bytes[1] = m_bulkReadCommand[Packet::PARAMETER + 1 + (i * 3) + 1];
            bytes[2] = m_bulkReadCommand[Packet::PARAMETER + 1 + (i * 3) + 2];
            
            // Erase our 3 bytes for this packet
            m_bulkReadCommand.erase(std::begin(m_bulkReadCommand) + Packet::PARAMETER + 1 + (i * 3),
                                    std::begin(m_bulkReadCommand) + Packet::PARAMETER + 1 + (i * 3) + 3);
            
            // Insert our 3 bytes at the end
            m_bulkReadCommand.insert(std::end(m_bulkReadCommand) - 1, bytes, bytes + 3);
            
            firstError = false;
            
            // Set for motor data
            if(r.header.id >= ID::R_SHOULDER_PITCH && r.header.id <= ID::HEAD_TILT)
                memset(&data.motors[r.header.id - 1], 0xFF, sizeof(Types::MX28Data));
            
            // Set for FSR data
            else if(r.header.id == ID::R_FSR || r.header.id == ID::L_FSR)
                memset(data.fsr + (r.header.id - ID::R_FSR), 0xFF, sizeof(Types::FSRData));
            
            // Set CM730 data
            else if(r.header.id == ID::CM730)
                memset(&data, 0xFF, sizeof(Types::CM730Data));
        }
        
        // This data could just be bad data because of previous sensor errors, only the first is reliable
        else {
            // Set for motor data
            if(r.header.id >= ID::R_SHOULDER_PITCH && r.header.id <= ID::HEAD_TILT)
                memset(&data.motors[r.header.id - 1], 0xFF, sizeof(Types::MX28Data));
            
            // Set for FSR data
            else if(r.header.id == ID::R_FSR || r.header.id == ID::L_FSR)
                memset(data.fsr + (r.header.id - ID::R_FSR), 0xFF, sizeof(Types::FSRData));
            
            // Set CM730 data
            else if(r.header.id == ID::CM730)
                memset(&data, 0xFF, sizeof(Types::CM730Data));
        }
    }
    
    return data;
}

void Darwin::Darwin::writeMotors(const std::vector<Types::MotorValues>& motors) {
    
    // Check that our MotorValues object is the correct size (the difference + 1 + another for the id)
    static_assert(sizeof(Types::MotorValues) == MX28::Address::TORQUE_LIMIT_H - MX28::Address::TORQUE_ENABLE + 2,
                  "The MotorValues type is the wrong size");
    
    // We allocate 8 bytes for normal things, and then space for all the motor values
    std::vector<uint8_t> packet;
    packet.resize(8 + (motors.size() * sizeof(Types::MotorValues)));
    
    // Build our packet
    packet[Packet::MAGIC] = 0xFF;
    packet[Packet::MAGIC + 1] = 0xFF;
    packet[Packet::ID] = ID::BROADCAST; // Broadcast id
    packet[Packet::LENGTH] = 4 + (motors.size() * sizeof(Types::MotorValues));
    packet[Packet::INSTRUCTION] = DarwinDevice::Instruction::SYNC_WRITE;
    
    // Our start address (we start at torque enable)
    packet[Packet::PARAMETER] = MX28::Address::TORQUE_ENABLE;
    // Our data length (not including our ID)
    packet[Packet::PARAMETER + 1] = sizeof(Types::MotorValues) - 1;
    
    // Our motor values
    memcpy(&packet[Packet::PARAMETER + 2], motors.data(), motors.size());
    
    // Our checksum
    packet.back() = calculateChecksum(packet.data());
    
    // Execute the command
    m_uart.executeBroadcast(packet);
}