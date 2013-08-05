#include "DarwinPlatform.h"

#include <armadillo>

namespace modules {
    
    DarwinPlatform::DarwinPlatform(NUClear::PowerPlant& plant) : Reactor(plant), m_darwin("/dev/ttyUSB0") {
        
        // This trigger gets the sensor data from the CM730
        on<Trigger<Every<20, std::chrono::milliseconds>>>([this](const time_t& time) {
            
            // Read our data
            Darwin::BulkReadResults data = m_darwin.bulkRead();
            
            Messages::DarwinSensors* sensors = new Messages::DarwinSensors;
            
            // LED Pannel
            bool LEDPanel2 = (data.cm730.ledPanel & 0x01) == 0x01;
            bool LEDPanel3 = (data.cm730.ledPanel & 0x02) == 0x02;
            bool LEDPanel4 = (data.cm730.ledPanel & 0x04) == 0x04;
            
            // Colored LEDs (in 24 bit RGB)
            uint8_t headLED[3] = {
                static_cast<uint8_t>((data.cm730.headLED & 0x7C00) >> 7),
                static_cast<uint8_t>((data.cm730.headLED & 0x03E0) >> 2),
                static_cast<uint8_t>((data.cm730.headLED & 0x001F) << 3)
            };
            uint8_t eyeLED[3] = {
                static_cast<uint8_t>((data.cm730.eyeLED & 0x7C00) >> 7),
                static_cast<uint8_t>((data.cm730.eyeLED & 0x03E0) >> 2),
                static_cast<uint8_t>((data.cm730.eyeLED & 0x001F) << 3)
            };
            
            // Buttons
            bool startButton = (data.cm730.buttons & 0x02) == 0x02;
            bool modeButton = (data.cm730.buttons & 0x01) == 0x01;
            
            // Voltage (in volts)
            float voltage = data.cm730.voltage / 10.0;
            
            // 0 is -4g, 512 is 0g, 1023 is +4g
            
            // Acceleronometer (in m/s)
            const double ACCELERONOMETER_CONVERSION = 0.07661445312; // 512 * x = 9.80665 * 4
            arma::vec3 acceleronometer = {
                (data.cm730.acceleronometer.x - 512) * ACCELERONOMETER_CONVERSION,
                (data.cm730.acceleronometer.y - 512) * ACCELERONOMETER_CONVERSION,
                (data.cm730.acceleronometer.z - 512) * ACCELERONOMETER_CONVERSION
            };
            
            // Gyroscope (measured in Radians per second)
            const double GYROSCOPE_CONVERSION = 0.0545415391248228; // 512 * x = 80PI/9 (1600 degrees in radians)
            arma::vec3 gyroscope = {
                (data.cm730.gyroscope.x - 512) * GYROSCOPE_CONVERSION,
                (data.cm730.gyroscope.x - 512) * GYROSCOPE_CONVERSION,
                (data.cm730.gyroscope.x - 512) * GYROSCOPE_CONVERSION 
            };
            
            
            // In normalized value (0-1)
            float leftMic = data.cm730.adc[0] / 1024.0;
            float rightMic = data.cm730.adc[8] / 1024.0;
            
            // TODO before every read, check to see if all the values are 0xFF, this means an error
            
        // FORCE SENSITIVE RESISTORS
            
            // FSR in Newtons
            float rightFSR1 = data.fsr[0].fsr1 / 1000;
            float rightFSR2 = data.fsr[0].fsr2 / 1000;
            float rightFSR3 = data.fsr[0].fsr3 / 1000;
            float rightFSR4 = data.fsr[0].fsr4 / 1000;
            
            // Normalized foot centre
            float rightFSRCentreX = data.fsr[0].centreX == 0xFF ? NAN : data.fsr[0].centreX / 254.0;
            float rightFSRCentreY = data.fsr[0].centreY == 0xFF ? NAN : data.fsr[0].centreY / 254.0;
            
            // FSR in Newtons
            float leftFSR1 = data.fsr[1].fsr1 / 1000;
            float leftFSR2 = data.fsr[1].fsr2 / 1000;
            float leftFSR3 = data.fsr[1].fsr3 / 1000;
            float leftFSR4 = data.fsr[1].fsr4 / 1000;
            
            // Normalized foot centre
            float leftFSRCentreX = data.fsr[1].centreX == 0xFF ? NAN : data.fsr[1].centreX / 254.0;
            float leftFSRCentreY = data.fsr[1].centreY == 0xFF ? NAN : data.fsr[1].centreY / 254.0;
            
        // MOTORS
            
            bool motorTorqueEnabled = data.motors[0].torqueEnabled;
            bool motorLED = data.motors[0].LED;
            
            float dGain = data.motors[0].dGain / 254.0;
            float iGain = data.motors[0].iGain / 254.0;
            float pGain = data.motors[0].pGain / 254.0;
            
            // In radians
            const double POSITION_CONVERSION = 0.00153588974; // 0.088 degrees in radians
            float goalPosition = data.motors[0].goalPosition * POSITION_CONVERSION;
            
            // In radians per second
            const double SPEED_CONVERSION = 0.01193805208; // 0.114 revolutions per minute in radians per second
            float movingSpeed = data.motors[0].movingSpeed * 0.01193805208;
            
            // Normalized between 0 and 1
            float torqueLimit = data.motors[0].torqueLimit / 1023.0;
            
            // In radians
            float position = data.motors[0].position * POSITION_CONVERSION;
            
            // In radians per second Positive is clockwise
            float presentSpeed = ((data.motors[0].speed & 0x3FF) * SPEED_CONVERSION) * (((data.motors[0].speed & 0x400) == 0x400) ? -1 : 1);
            
            // Normalized between -1 and 1 Positive is clockwise
            float presentLoad = ((data.motors[0].load & 0x3FF) / 1023.0) * (((data.motors[0].load & 0x400) == 0x400) ? -1 : 1);
            
            // In volts
            float motorVoltage = data.motors[0].voltage / 10.0;
            
            // In degrees c
            uint8_t temperature = data.motors[0].temperature;
        });
        
        // This trigger writes the motor positions to the Motors
        on<Trigger<Every<20, std::chrono::milliseconds>>, With<Messages::DarwinMotors>>([](const time_t& time, const Messages::DarwinMotors& motors) {
            
            // TODO convert DarwinMotors into our motor type
            
            std::vector<Darwin::Types::MotorValues> values;
            
            values.push_back({
                0, // Motor ID
                true, // Torque Enabled
                true, // LED On
                0 * 254, // D Gain
                0 * 254, // I Gain
                0 * 254, // P Gain
                0x00,   // Reserved Byte
                0 / POSITION_CONVERSION, // Goal Position
                0 / SPEED_CONVERSION, // TODO factor in movement direction
                0 * 1023 // Torque Limit
            });
            
            uint8_t motorId;
            bool torqueEnabled;
            bool ledOn;
            uint8_t dGain;
            uint8_t iGain;
            uint8_t pGain;
            uint8_t reserved;
            uint16_t goalPostion;
            uint16_t movingSpeed;
            uint16_t torqueLimit;
        });
    }
}