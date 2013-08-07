#ifndef MESSAGES_DARWINSENSORS_H
#define MESSAGES_DARWINSENSORS_H

#include <armadillo>

namespace Messages {

    /**
     * TODO
     */
    struct DarwinSensors {

        enum Error {
            OK              = 0x0000,
            INPUT_VOLTAGE   = 0x0001,
            ANGLE_LIMIT     = 0x0002,
            OVERHEATING     = 0x0004,
            RANGE           = 0x0008,
            CHECKSUM        = 0x0010,
            OVERLOAD        = 0x0020,
            INSTRUCTION     = 0x0040,
            CORRUPT_DATA    = 0x0080,
            TIMEOUT         = 0x0100
        };

        uint8_t cm730ErrorFlags;

        struct LEDPanel {
            bool led2;
            bool led3;
            bool led4;
        } ledPanel;

        struct ColouredLED {
            uint8_t r;
            uint8_t g;
            uint8_t b;
        };

        struct HeadLED : public ColouredLED {
        } headLED;

        struct EyeLED : public ColouredLED {
        } eyeLED;

        struct Buttons {
            bool left;
            bool middle;
        } buttons;

        float voltage;

        arma::vec3 acceleronometer;
        arma::vec3 gyroscope;

        struct FSR {
            float fsr1;
            float fsr2;
            float fsr3;
            float fsr4;

            float centreX;
            float centreY;
            
            uint8_t errorFlags;
        };

        struct FSRs {
            FSR left;
            FSR right;
        } fsr;

        struct Servo {
            enum ID {
                HEAD_PAN = 0,
                HEAD_TILT = 1,
                R_SHOULDER_PITCH = 2,
                L_SHOULDER_PITCH = 3,
                R_SHOULDER_ROLL = 4,
                L_SHOULDER_ROLL = 5,
                R_ELBOW = 6,
                L_ELBOW = 7,
                R_HIP_YAW = 8,
                L_HIP_YAW = 9,
                R_HIP_ROLL = 10,
                L_HIP_ROLL = 11,
                R_HIP_PITCH = 12,
                L_HIP_PITCH = 13,
                R_KNEE = 14,
                L_KNEE = 15,
                R_ANKLE_PITCH = 16,
                L_ANKLE_PITCH = 17,
                R_ANKLE_ROLL = 18,
                L_ANKLE_ROLL = 19
            };

            uint8_t errorFlags;

            bool torqueEnabled;
            bool led;

            float pGain;
            float iGain;
            float dGain;

            float goalPosition;
            float movingSpeed;
            float torqueLimit;

            float presentPosition;
            float presentSpeed;

            float load;
            float voltage;
            uint8_t temperature;
        };
        
        struct Servos {
            Servo rShoulderPitch;
            Servo lShoulderPitch;
            Servo rShoulderRoll;
            Servo lShoulderRoll;
            Servo rElbow;
            Servo lElbow;
            Servo rHipYaw;
            Servo lHipYaw;
            Servo rHipRoll;
            Servo lHipRoll;
            Servo rHipPitch;
            Servo lHipPitch;
            Servo rKnee;
            Servo lKnee;
            Servo rAnklePitch;
            Servo lAnklePitch;
            Servo rAnkleRoll;
            Servo lAnkleRoll;
            Servo headPan;
            Servo headTilt;

            Servo& operator[](int index);
        } servo;
    };
};

#endif