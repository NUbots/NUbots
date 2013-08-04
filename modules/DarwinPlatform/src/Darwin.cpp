#include "Darwin.h"

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
    
    // Run a self test (test which sensors are working and which are not)
    selfTest();
    
    buildBulkReadPacket();
}

void Darwin::Darwin::selfTest() {
    
    // Ping all our motors
    for (int i = 0; i < 20; ++i) {
        (&m_rShoulderPitch)[i].ping();
    }
    
    // Ping our two FSRs
    m_lFSR.ping();
    m_rFSR.ping();
}

void Darwin::Darwin::buildBulkReadPacket() {
    std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> request;
    
    // The CM730 read from the LED_PANNEL, until the ADC15 (last available byte)
    request.push_back(std::make_tuple(200, CM730::Address::LED_PANNEL, CM730::Address::ADC15_H - CM730::Address::LED_PANNEL + 1));
    
    // Double check that our type is big enough to hold the result
    static_assert(sizeof(Types::CM730Data) == CM730::Address::ADC15_H - CM730::Address::LED_PANNEL + 1,
                  "The CM730 type is the wrong size");
    
    // All of the motors
    for (int i = 1; i <= 20; ++i) {
        request.push_back(std::make_tuple(i, MX28::Address::TORQUE_ENABLE, MX28::Address::PRESENT_TEMPERATURE - MX28::Address::TORQUE_ENABLE + 1));
    }
    
    // Double check that our type is big enough to hold the result
    static_assert(sizeof(Types::MX28Data) == MX28::Address::PRESENT_TEMPERATURE - MX28::Address::TORQUE_ENABLE + 1,
                  "The MX28 type is the wrong size");
    
    // The Two FSRs
    request.push_back(std::make_tuple(111, FSR::Address::FSR1_L, FSR::Address::FSR_Y - FSR::Address::FSR1_L + 1));
    request.push_back(std::make_tuple(112, FSR::Address::FSR1_L, FSR::Address::FSR_Y - FSR::Address::FSR1_L + 1));
    
    // Double check that our type is big enough to hold the result
    static_assert(sizeof(Types::FSRData) == FSR::Address::FSR_Y - FSR::Address::FSR1_L + 1,
                  "The FSR type is the wrong size");
    
    // Our actual packet we are sending
    std::vector<uint8_t> command(7 + (request.size() * 3));
    command[Packet::MAGIC] = 0xFF;
    command[Packet::MAGIC + 1] = 0xFF;
    command[Packet::ID] = 254;
    command[Packet::LENGTH] = 3 + (request.size() * 3);
    command[Packet::INSTRUCTION] = DarwinDevice::Instruction::BULK_READ;
    command[Packet::PARAMETER] = 0x00;
    
    // Copy our parameters in
    memcpy(&command.data()[Packet::PARAMETER + 1], request.data(), request.size() * 3);
    
    // Calculate our checksum
    command.back() = calculateChecksum(command.data());
    
    // Swap our command into the actual command location
    m_bulkReadCommand.swap(command);
}

Darwin::BulkReadResults Darwin::Darwin::bulkRead() {
    
    std::vector<CommandResult> results = m_uart.executeBulk(m_bulkReadCommand);
    
    BulkReadResults data;
    
    for(const auto& r : results) {
        if(r.header.errorcode == ErrorCode::NONE) {
            
            // If our response packet is a motor then copy it to the correct motor data location
            if(r.header.id >= ID::R_SHOULDER_PITCH && r.header.id <= ID::HEAD_TILT) {
                
                memcpy(&data.motors[r.header.id - 1], r.data.data(), r.data.size());
            }
            
            // If our response packet is for the FSRs
            else if(r.header.id == ID::R_FSR || r.header.id == ID::L_FSR) {
                
                // Copy our data into the appropriate FSR location
                memcpy(data.fsr + (r.header.id - ID::R_FSR), r.data.data(), r.data.size());
            }
            
            // If our response packet is for the CM730
            else if(r.header.id == ID::CM730) {
                
                // Copy our CM730 data into the start of the packet
                memcpy(&data, r.data.data(), r.data.size());
            }
        }
        else {
            // TODO swap this to the end of the vector (before the checksum)
        }
    }
    
    return data;
}

void Darwin::Darwin::writeMotors(const std::vector<Types::MotorValues>& motors) {
    
    // Check that our MotorValues object is the correct size (the difference + 1 + another for the id)
    static_assert(sizeof(Types::MotorValues) == MX28::Address::TORQUE_LIMIT_H - MX28::Address::TORQUE_ENABLE + 2,
                  "The MotorValues type is the wrong size");
    
    std::vector<uint8_t> packet;
    
    // We allocate 8 bytes for normal things, and then space for all the motor values
    packet.resize(8 + (motors.size() * sizeof(Types::MotorValues)));
    
    // Our magic bytes
    packet[Packet::MAGIC] = 0xFF;
    packet[Packet::MAGIC + 1] = 0xFF;
    
    // Our target id
    packet[Packet::ID] = 254; // Broadcast id
    
    // Our data length
    packet[Packet::LENGTH] = 4 + (motors.size() * sizeof(Types::MotorValues));
    
    // Our instruction
    packet[Packet::INSTRUCTION] = DarwinDevice::Instruction::SYNC_WRITE;
    
    // Our start address (we start at torque enable)
    packet[Packet::PARAMETER] = MX28::Address::TORQUE_ENABLE;
    // Our data length
    packet[Packet::PARAMETER + 1] = sizeof(Types::MotorValues);
    
    // Our motor values
    memcpy(&packet[Packet::PARAMETER + 2], motors.data(), motors.size());
    
    // Our checksum
    packet.back() = calculateChecksum(packet.data());
    
    // Execute the command
    m_uart.executeBroadcast(packet);
}