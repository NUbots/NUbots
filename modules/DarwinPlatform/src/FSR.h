#ifndef FSR_H
#define FSR_H

#include "DarwinDevice.h"

namespace Darwin
{
	class FSR : public DarwinDevice {
        
    public:
        
        enum Address
        {
            MODEL_NUMBER_L            = 0,
            MODEL_NUMBER_H            = 1,
            VERSION                   = 2,
            ID                        = 3,
            BAUD_RATE                 = 4,
            RETURN_DELAY_TIME         = 5,
            RETURN_LEVEL              = 16,
            OPERATING_MODE            = 19,
            LED                       = 25,
            FSR1_L                    = 26,
            FSR1_H                    = 27,
            FSR2_L                    = 28,
            FSR2_H                    = 29,
            FSR3_L                    = 30,
            FSR3_H                    = 31,
            FSR4_L                    = 32,
            FSR4_H                    = 33,
            FSR_X                     = 34,
            FSR_Y                     = 35,
            PRESENT_VOLTAGE           = 42,
            REGISTERED_INSTRUCTION    = 44,
            LOCK                      = 47
        };
        
        FSR(UART& coms, int id);
    };
}

#endif
