/*
 * This is some dark magic, probably just close the file
 */

#include "DynamixelProtocol.hpp"

// Communication Result
#define COMM_SUCCESS 0         // tx or rx packet communication success
#define COMM_RX_TIMEOUT -3001  // There is no status packet
#define COMM_RX_CORRUPT -3002  // Incorrect status packet
#define COMM_TX_FAIL -1001     // Incorrect instruction packet

#define LATENCY_TIMER 16  // Random value taken from SDK

#define BYTE_WAIT 1200      // Random value from NUbots code
#define PACKET_WAIT 100000  // Random value from NUbots code

#define UART_DEVICE "/dev/ttyUSB0"
#define UART_BAUD 115200

class utility::io::uart uart;

UART::UART() : device(UART_DEVICE), baud(UART_BAUD) {
    std::cout << "Initilising UART" << std::endl;
    uart.open(device, baud);
    tx_time_per_byte = (1000.0 / (double) baud) * 10.0;
    std::cout << "Baud rate set to: " << baud << std::endl;
}

// Function that takes servo ID, address, and data
// Calls dynamixel execute functions
template <typename T>
int executeWriteSingle(uint8_t servo_ID, uint16_t address, const T& data) {

    std::cout << "Writing servo " << (int) servo_ID << " to " << data << std::endl;

    auto buf = dynamixel::v2::WriteCommand<T>(servo_ID, address, data);
    if (uart.good()) {
        uart.write(&buf, sizeof(buf));
        return 0;
    }
    return -1;
}

int executeWriteBuffer(uint8_t servo_ID, uint16_t address, uint8_t* data, uint8_t* buf) {
    // *buf = dynamixel::v2::BulkWriteData<uint8_t>(servo_ID, address, *data);
    return 0;
}

template <typename T>
int executeWriteMulti(uint8_t servo_ID, uint16_t address, const T& data, uint count) {
    // auto group_buf = dynamixel::v2::BulkWriteCommand<uint8_t, sizeof(buf)>(&buf, sizeof(buf));
    // if (uart.good()) {
    //     for (int i = 0; i < count; i++) {
    //         if (executeWriteSingle(servo_ID[i], address, data[i]) != 0) {
    //             std::cout << "ERROR: Could not write to servo" << std::endl;
    //         }
    //     }
    //     return 0;
    // }
    return -1;
}

template <typename T>
int executeReadSingle(uint8_t servo_ID, uint16_t address, uint16_t size, T& rx_data) {

    auto tx_buf = dynamixel::v2::ReadCommand(servo_ID, address, size);
    dynamixel::v2::StatusReturnCommand<T> stat;

    int rx_result      = COMM_TX_FAIL;
    uint16_t rx_length = 0;
    uint8_t magic[4]   = {0xFF, 0xFF, 0xFD, 0x00};

    // Check that our UART is still connected
    if (uart.good()) {
        // Now lets write our packet
        uart.write(&tx_buf, sizeof(tx_buf));
        // First we find the packet magic number in order to sync with the channel
        setPacketTimeout((uint16_t)(PACKET_WAIT));
        for (int sync = 0; sync < 4;) {
            if (isPacketTimeout() != true) {
                uint8_t byte;
                if (uart.read(&byte, 1) > 0) {
                    for (int i = 0; i < 4; i++) {
                        if (byte == magic[i]) {
                            sync = i + 1;
                            break;
                        }
                    }
                }
            }
            else {
                // The result is pre initialized as a timeout
                std::cout << "ERROR: Failed to sync servo " << (int) servo_ID << std::endl;
                return COMM_RX_TIMEOUT;
            }
        }

        // We now are now waiting for 4 bytes
        setPacketTimeout(
            (uint16_t)((BYTE_WAIT * (sizeof(stat.magic) + 2)) + (BYTE_WAIT * size) + (2000) + PACKET_WAIT));
        while (true) {
            if (isPacketTimeout() != true) {
                rx_length += uart.read((reinterpret_cast<uint8_t*>(&stat) + sizeof(stat.magic)),
                                       sizeof(stat) - sizeof(stat.magic) - rx_length);
                if (rx_length == sizeof(stat) - sizeof(stat.magic)) {
                    break;
                }
            }
            else {
                // The result is pre initialized as a timeout
                std::cout << "ERROR: Failed to read packet " << rx_length << " of " << sizeof(stat) - sizeof(stat.magic)
                          << " servo " << (int) servo_ID << std::endl;
                return COMM_RX_TIMEOUT;
            }
        }
        // Validate our checksum
        uint16_t crc = dynamixel::v2::calculateChecksum(&stat, 0);
        if (stat.checksum != crc) {
            std::cout << std::hex << "ERROR: Checksum corrupt " << stat.checksum << " calculated " << crc << " servo "
                      << (int) servo_ID << std::dec << std::endl;
            std::cout << std::hex << std::endl;
            std::cout << " stat.magic       " << (int) stat.magic << std::endl;
            std::cout << " stat.id          " << (int) stat.id << std::endl;
            std::cout << " stat.length      " << (int) stat.length << std::endl;
            std::cout << " stat.instruction " << (int) stat.instruction << std::endl;
            std::cout << " stat.error       " << (int) stat.error << std::endl;
            std::cout << " stat.data        " << (int) stat.data << std::endl;
            std::cout << " stat.checksum    " << (int) stat.checksum << std::endl;
            std::cout << std::dec << std::endl;
            std::array<uint8_t, 12> test = *(reinterpret_cast<std::array<uint8_t, 12>*>(&stat));
            for (int k = 0; k < 12; k++) {
                std::cout << ((int) (test[k])) << std::endl;
            }
            return COMM_RX_CORRUPT;
        }

        // Return the packet we recieved
        rx_data = stat.data;
        return COMM_SUCCESS;
    }
}

template <typename T>
int executeReadMulti(uint8_t servo_ID, uint16_t address, uint16_t size, T& rx_data, uint count) {
    // auto buf = dynamixel::v2::BulkRead();
    // if (uart.good()) {
    //     uart.read(&buf, sizeof(buf));
    // 		return 0;
    // }
    // for (int i = 0; i < count; i++) {
    //     executeReadSingle(uint8_t servo_ID[i], uint16_t address, uint16_t size, T & rx_data);
    //     delay(10);
    // }
    return -1;
}


void setPacketTimeout(uint16_t packet_length) {
    packet_start_time_ = getCurrentTime();
    packet_timeout_    = (tx_time_per_byte * (double) packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
}

void setPacketTimeout(double msec) {
    packet_start_time_ = getCurrentTime();
    packet_timeout_    = msec;
}

bool isPacketTimeout() {
    if (getTimeSinceStart() > packet_timeout_) {
        packet_timeout_ = 0;
        return true;
    }

    return false;
}

double getCurrentTime() {
    return (double) millis();  // From wiringPi Library
}

double getTimeSinceStart() {
    double elapsed_time;
    elapsed_time = getCurrentTime() - packet_start_time_;
    if (elapsed_time < 0.0) packet_start_time_ = getCurrentTime();

    return elapsed_time;
}

void Dynamixel_init() {
    // Report back on any servos that failed to connect
    // Set gains and torque
    uint8_t data;
    for (int servo_ID = 1; servo_ID < 6; servo_ID++) {
        // TODO This should probably be a ping, but i dont think i have a function to handle it
        if (executeReadSingle(servo_ID, MX28_ADDRESS_VALUE(ID), MX28_SIZE_VALUE(ID), data) == COMM_SUCCESS) {
            delay(10);
            executeWriteSingle(servo_ID, MX28_ADDRESS_VALUE(TORQUE_ENABLE), 1);
            delay(10);
            // executeWriteSingle(servo_ID, MX28_ADDRESS_VALUE(POSITION_P_GAIN), 16000);
            // delay(10);
        }
        else {
            std::cout << "ERROR: Failed to ping servo " << (int) servo_ID << std::endl;
        }
    }
    for (int servo_ID = 6; servo_ID < 8; servo_ID++) {
        // TODO This should probably be a ping, but i dont think i have a function to handle it
        if (executeReadSingle(servo_ID, MX64_ADDRESS_VALUE(ID), MX64_SIZE_VALUE(ID), data) == COMM_SUCCESS) {
            delay(10);
            executeWriteSingle(servo_ID, MX64_ADDRESS_VALUE(TORQUE_ENABLE), 1);
            delay(10);
            // executeWriteSingle(servo_ID, MX64_ADDRESS_VALUE(VELOCITY_P_GAIN), 16000);
            // delay(10);
        }
        else {
            std::cout << "ERROR: Failed to ping servo " << (int) servo_ID << std::endl;
        }
    }
}

template int executeReadSingle<uint8_t>(uint8_t, uint16_t, uint16_t, uint8_t&);
template int executeReadSingle<uint16_t>(uint8_t, uint16_t, uint16_t, uint16_t&);
template int executeReadSingle<uint32_t>(uint8_t, uint16_t, uint16_t, uint32_t&);
template int executeReadSingle<int32_t>(uint8_t, uint16_t, uint16_t, int32_t&);

template int executeWriteSingle<uint8_t>(uint8_t, uint16_t, const uint8_t&);
template int executeWriteSingle<uint16_t>(uint8_t, uint16_t, const uint16_t&);
template int executeWriteSingle<uint32_t>(uint8_t, uint16_t, const uint32_t&);
