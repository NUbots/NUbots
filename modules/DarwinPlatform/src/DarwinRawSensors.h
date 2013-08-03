#ifndef MESSAGES_DARWINRAWSENSORS_H
#define MESSAGES_DARWINRAWSENSORS_H

#include <stdint.h>

namespace Darwin {
    
#pragma pack(push, 1) // Here we disable the OS putting in padding bytes so we can raw memcpy into this data
    namespace Types {
        
        struct Gyro {
            int16_t z;
            int16_t y;
            int16_t x;
        };
        
        struct Accelerometer {
            int16_t x;
            int16_t y;
            int16_t z;
        };
        
        struct MX28Data {
            bool torqueEnabled;
            bool LED;
            uint8_t dGain;
            uint8_t iGain;
            uint8_t pGain;
            uint8_t reservedByte;
            uint16_t goalPosition;
            uint16_t movingSpeed;
            uint16_t torqueLimit;
            uint16_t position;
            uint16_t speed;
            uint16_t load;
            uint8_t voltage;
            uint8_t temperature;
        };
        
        struct FSRData {
            uint16_t fsr1;
            uint16_t fsr2;
            uint16_t fsr3;
            uint16_t fsr4;
            uint8_t centreX;
            uint8_t centreY;
        };
        
        struct CM730Data {
            uint8_t ledPanel;
            uint16_t headLED;
            uint16_t eyeLED;
            uint8_t buttons;
            uint8_t reserved[7];
            Gyro gyroscope;
            Accelerometer acceleronometer;
            uint8_t voltage;
            uint16_t adc[15];
        };
        
        struct MotorValues {
            bool torqueEnabled;
            bool ledOn;
            uint8_t dGain;
            uint8_t iGain;
            uint8_t pGain;
            uint16_t goalPostion;
            uint16_t movingSpeed;
            uint16_t torqueLimit;
        };
#pragma pack(pop)
    }
    
    struct BulkReadResults {
        Types::CM730Data cm730;
        Types::MX28Data motors[20];
        Types::FSRData fsr[2];
    };
    
}

#endif