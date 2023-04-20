#include <fmt/format.h>

#include "Convert.hpp"
#include "HardwareIO.hpp"

#include "utility/math/comparison.hpp"

namespace module::platform::openCR {

    using message::platform::RawSensors;
    using message::platform::StatusReturn;

    /*
        Process the status return packet data
    */

    void HardwareIO::process_model_information(const StatusReturn& packet) {
        log<NUClear::DEBUG>(
            fmt::format("Model {:02X} {:02X}, Version {:02X}", packet.data[1], packet.data[0], packet.data[2]));
        uint16_t model  = (packet.data[1] << 8) | packet.data[0];
        uint8_t version = packet.data[2];
        log<NUClear::INFO>(fmt::format("OpenCR Model...........: {:#06X}", model));
        log<NUClear::INFO>(fmt::format("OpenCR Firmware Version: {:#04X}", version));
    }

    void HardwareIO::process_opencr_data(const StatusReturn& packet) {
        log<NUClear::TRACE>("processOpenCRData START");

        const OpenCRReadData data = *(reinterpret_cast<const OpenCRReadData*>(packet.data.data()));

        log<NUClear::TRACE>("\tOpenCRReadData casted");

        // 00000321
        // LED_1 = 0x01
        // LED_2 = 0x02
        // LED_3 = 0x04
        opencrState.ledPanel = {bool(data.led & 0x01), bool(data.led & 0x02), bool(data.led & 0x04)};

        // 0BBBBBGG GGGRRRRR
        // R = 0x001F
        // G = 0x02E0
        // B = 0x7C00
        uint32_t RGB = 0;
        RGB |= uint8_t(data.rgbLed & 0x001F) << 16;  // R
        RGB |= uint8_t(data.rgbLed & 0x02E0) << 8;   // G
        RGB |= uint8_t(data.rgbLed & 0x7C00);        // B
        opencrState.headLED = {RGB};

        // 00004321
        // Button_4 = Not used
        // Button Right (Reset) = 0x01
        // Button Middle = 0x02
        // Button Left = 0x04
        opencrState.buttons = {bool(data.button & 0x04), bool(data.button & 0x02), bool(data.button & 0x01)};

        opencrState.gyro = Eigen::Vector3f(convert::gyro(data.gyro[2]),    // X
                                           convert::gyro(data.gyro[1]),    // Y
                                           -convert::gyro(data.gyro[0]));  // Z

        opencrState.acc = Eigen::Vector3f(convert::acc(data.acc[0]),   // X
                                          convert::acc(data.acc[1]),   // Y
                                          convert::acc(data.acc[2]));  // Z

        // Command send/receive errors only
        opencrState.alertFlag   = static_cast<bool>(packet.alert);
        opencrState.errorNumber = static_cast<int>(packet.error);

        // Work out a battery charged percentage
        batteryState.currentVoltage = convert::voltage(data.voltage);
        float percentage            = std::max(0.0f,
                                    (batteryState.currentVoltage - batteryState.flatVoltage)
                                        / (batteryState.chargedVoltage - batteryState.flatVoltage));

        // Battery percentage has changed, recalculate LEDs
        if (!utility::math::almost_equal(percentage, batteryState.percentage, 2)) {
            batteryState.dirty      = true;
            batteryState.percentage = percentage;

            uint32_t ledr            = 0;
            std::array<bool, 3> ledp = {false, false, false};

            if (batteryState.percentage > 0.90f) {
                ledp = {true, true, true};
                ledr = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
            }
            else if (batteryState.percentage > 0.70f) {
                ledp = {false, true, true};
                ledr = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
            }
            else if (batteryState.percentage > 0.50f) {
                ledp = {false, false, true};
                ledr = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
            }
            else if (batteryState.percentage > 0.30f) {
                ledp = {false, false, false};
                ledr = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
            }
            else if (batteryState.percentage > 0.20f) {
                ledp = {false, false, false};
                ledr = (uint8_t(0xFF) << 16) | (uint8_t(0x00) << 8) | uint8_t(0x00);
            }
            else if (batteryState.percentage > 0) {
                ledp = {false, false, false};
                ledr = (uint8_t(0xFF) << 16) | (uint8_t(0x00) << 8) | uint8_t(0x00);
            }
            // Error in reading voltage blue
            else {
                ledp = {false, false, false};
                ledr = (uint8_t(0x00) << 16) | (uint8_t(0x00) << 8) | uint8_t(0xFF);
            }
            emit(std::make_unique<RawSensors::LEDPanel>(ledp[2], ledp[1], ledp[0]));
            emit(std::make_unique<RawSensors::HeadLED>(ledr));
        }

        log<NUClear::TRACE>("processOpenCRData END");
    }

    void HardwareIO::process_servo_data(const StatusReturn& packet) {
        log<NUClear::TRACE>("processServoData START");
        const DynamixelServoReadData data = *(reinterpret_cast<const DynamixelServoReadData*>(packet.data.data()));
        // IDs are 1..20 so need to be converted for the servoStates index
        uint8_t servoIndex = packet.id - 1;

        servoStates[servoIndex].torqueEnabled = (data.torqueEnable == 1);
        // Servo error status, NOT dynamixel status packet error.
        servoStates[servoIndex].errorFlags     = data.hardwareErrorStatus;
        servoStates[servoIndex].presentPWM     = convert::PWM(data.presentPWM);
        servoStates[servoIndex].presentCurrent = convert::current(data.presentCurrent);
        // warning: no idea if the conversion below is correct, just trusting the existing
        // conversion functions in the branch. Whatever is happening it's very different to
        // the solution in the CM740 hwIO (which does make sense). Probably need to test IRL
        // to understand what's going on.
        servoStates[servoIndex].presentVelocity = convert::velocity(data.presentVelocity);
        servoStates[servoIndex].presentPosition =
            convert::position(servoIndex, data.presentPosition, nugus.servo_direction, nugus.servo_offset);
        servoStates[servoIndex].voltage     = convert::voltage(data.presentVoltage);
        servoStates[servoIndex].temperature = convert::temperature(data.presentTemperature);
        log<NUClear::TRACE>("processServoData END");
    }

    // void HardwareIO::process_fsr_data(const StatusReturn& packet) {
    //     // process packet once we know the structure
    //     return;
    // }


}  // namespace module::platform::openCR
