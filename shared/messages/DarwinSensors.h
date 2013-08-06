#ifndef MESSAGES_DARWINSENSORS_H
#define MESSAGES_DARWINSENSORS_H

#include <armadillo>

namespace Messages {
    
    /**
     * TODO
     */
    struct DarwinSensors {
        
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
            bool start;
            bool mode;
        } buttons;
        
        float voltage;
        
        arma::vec3 acceleronometer;
        arma::vec3 gyroscope;
        
        struct FSR {
            
        };
        
        struct FSRs {
            FSR left;
            FSR right;
        };
    };
};

#endif