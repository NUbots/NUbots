/* A file containing all of the definitions relating to the Dynamixel DX-117s
 
 Jason Kulk
 March 2009
 */
#ifndef DX117_H
#define DX117_H

// Almost all of these names are the same as that specified by the Direct Access API (there are a few exceptions where the API name made no sense)
// Most of the useful comments are my own, the rest was taken directly from the API header file DXL_packet.h

#define DX117_START_BYTE             0xFF

// Instruction Codes
#define DX117_PING                   0x01		    // Ping instruction
#define DX117_READ                   0x02		    // Read instruction
#define DX117_WRITE                  0x03		    // Write instruction
#define DX117_REG_WRITE              0x04		    // Register write instruction
#define DX117_ACTION                 0x05		    // Action instruction (execute the last registered write) 
#define DX117_RESET                  0x06		    // Reset instruction (changes the values in the control table back to the default values
#define DX117_SYNC_WRITE             0x83		    // Sync write instruction. I have no idea what this does but it is included in the Direct-Access API

// Alarm LED byte bit defintions
#define DX117_ALARM_VOLTAGE          0x01	        // 00000001	- DXL voltage error
#define DX117_ALARM_ANGLE	          0x02	        // 00000010	- DXL limited angle error
#define DX117_ALARM_OVERHEAT         0x04	        // 00000100	- DXL overheating error
#define DX117_ALARM_RANGE		       0x08	        // 00001000	- DXL range error
#define DX117_ALARM_CHECKSUM         0x10	        // 00010000	- DXL packet's checksum error
#define DX117_ALARM_OVERLOAD         0x20	        // 00100000	- DXL overload error
#define DX117_ALARM_INSTRUCTION 	    0x40	        // 01000000	- DXL instruction code error
#define DX117_ALARM_ALL              0x7F         // 01111111 - All errors will force the LED to blink

// Alarm Shutdown byte bit defintions
#define DX117_SHUTDOWN_VOLTAGE 	    0x01	        // 00000001	- DXL voltage error
#define DX117_SHUTDOWN_ANGLE         0x02	        // 00000010	- DXL limited angle error
#define DX117_SHUTDOWN_OVERHEAT 	    0x04	        // 00000100	- DXL overheatting error
#define DX117_SHUTDOWN_RANGE         0x08	        // 00001000	- DXL range error
#define DX117_SHUTDOWN_CHECKSUM 	    0x10	        // 00010000	- DXL packet's checksum error
#define DX117_SHUTDOWN_OVERLOAD      0x20	        // 00100000	- DXL overload error
#define DX117_SHUTDOWN_INSTRUCTION   0x40	        // 01000000	- DXL instruction code error

// Dynamixel Baud rate
#define DX117_BAUD_1M                0x01         // 1000000 bps
#define DX117_BAUD_56k               0x22         // 57600 bps (this is the default value)

// Status packet response settings
#define DX117_RETURN_NONE		       0x00		    // No Status packets are ever sent
#define DX117_RETURN_READ		       0x01		    // Status packet sent on read instruction only
#define DX117_RETURN_ALL 	          0x02		    // Every instruction is replied with a status packet

// Load direction
#define DX117_CCW_LOAD 		          0x00		    // Counter-clockwise load direction
#define DX117_CW_LOAD 	             0x01		    // clockwise load direction

// Registered state
#define DX117_REGISTER_DONE 	       0x00		    // Registered done
#define DX117_REGISTERED_INST        0x01		    // Registered instruction

// LED state
#define DX117_LED_OFF 	             0x00		    // Turn off the LED on the back on the dynamixel motor
#define DX117_LED_ON 	             0x01		    // Turn on LED

// Torque state 
#define DX117_TORQUE_OFF 	          0x00	        // Turn off Torque to the dynamixel. This puts the motor in free-wheel mode, which leaves the motor suprisingly free to rotate under external torques
#define DX117_TORQUE_ON 		       0x01	        // Turn on Torque for the dynamixel

// Moving state
#define DX117_NOT_MOVING 	          0x00		    // The motor is not moving under its own power, however, it could be moving due to an external torque
#define DX117_MOVING 	             0x01		    // The motor is moving under its own power

// Minimum/Maximum Range values
#define DX117_MAX_ID 	             0xFD         // the maximum ID allowed for a dynamixel. Remember that 0xFE is the broadcasting ID
#define DX117_MAX_TEMPERATURE        150          // the maximum temperature (in C) before shutdown occurs
#define DX117_MIN_VOLTAGE		       50           // the minimum input voltage (in 10 * Volts) before the motor shuts down
#define DX117_MAX_VOLTAGE		       250          // the maximum inpurt voltage (in 10 * Volts) before the motor shuts down
#define DX117_MAX_POSITION           0x03FF       // the maximum angular position of the motor 0x3FF  300 degrees
#define DX117_MAX_SPEED 		       0x03FF       // the maximum rotational speed of the motor 0x3FF  70 rpm
#define DX117_MAX_TORQUE 	          0x03FF       // the maximum torque; 0x3FF  100% maximum torque

// Other
#define DX117_BROADCASTING_ID        0xFE	        // the broadcasting ID; use have all of the motors act on the instruction at the same time

// Compliance Settings
#define DX117_SLOPE_1          0x04
#define DX117_SLOPE_2          0x08
#define DX117_SLOPE_3          0x10
#define DX117_SLOPE_4          0x20
#define DX117_SLOPE_5          0x40
#define DX117_SLOPE_6          0x80
#define DX117_SLOPE_7          0xFE

// EEPROM. These values remain unchanged when the motor is powered off
#define P_MODEL_NUMBER_L       0x00            // Model number lower byte address
#define P_MODOEL_NUMBER_H      0x01            // Model number higher byte address
#define P_VERSION              0x02            // DXL version address
#define P_ID                   0x03            // the address of the unique ID
#define P_BAUD_RATE            0x04            // DXL baudrate address
#define P_RETURN_DELAY_TIME    0x05            // Return delay time address. This is the time delay in the DX between receiving an instruction and sending a status packet
#define P_CW_ANGLE_LIMIT_L     0x06            // CW limited angle lower byte address
#define P_CW_ANGLE_LIMIT_H     0x07            // CW limited angle higher byte address
#define P_CCW_ANGLE_LIMIT_L    0x08            // CCW limited angle lower byte address
#define P_CCW_ANGLE_LIMIT_H    0x09            // CCW limited angle higher byte address
#define P_LIMIT_TEMPERATURE    0x0b            // The address of the temperature limit before the DX will shut down if the appropriate bit in the shutdown register is 1
#define P_LOWER_LIMIT_VOLTAGE  0x0c            // The lower limit of the input voltage range address
#define P_UPPER_LIMIT_VOLTAGE  0x0d            // The upper limit of the input voltage range address, these values are used to trigger shutdowns
#define P_MAX_TORQUE_L         0x0e            // The maximum torque lower byte address
#define P_MAX_TORQUE_H         0x0f            // The maximum torque higher byte address, these addresses are used to limit the torque as a percentage of the maximum
#define P_RETURN_LEVEL         0x10            // The status packet response setting address
#define P_ALARM_LED            0x11            // Alarm LED address
#define P_ALARM_SHUTDOWN       0x12            // Alarm shutdown address
#define P_DOWN_CALIBRATION_L   0x14            // Down calibration lower byte address
#define P_DOWN_CALIBRATION_H   0x15            // Down calibration higher byte address
#define P_UP_CALIBRATION_L     0x16            // Up calibration lower byte address
#define P_UP_CALIBRATION_H     0x17            // Up calibration higher byte address

// RAM
#define P_TORQUE_ENABLE        0x18            // This bit puts the motor into free-wheel mode. Write TORQUE_ON, TORQUE_OFF to this address
#define P_LED                  0x19            // LED on/off flag address. Write LED_ON and LED_OFF to turn the LED on the motors on and off
#define P_REGISTERED_INSTRUCTION 0x2c          // Registered instruction address. This flag indicates whether there has been an unexecuted register write. The action command resets this flag
#define P_MOVING               0x2e            // Moving state flag address. Compare with MOVING and NOT_MOVING to see whether the motor is moving under its own power.
#define P_LOCK                 0x2f            // Control table lock flag address. This flag prevents all but 0x18 to 0x23 from being modified. The lock can only be removed by powering down.

// Compliance Settings
#define P_CW_COMPLIANCE_MARGIN 0x1a            // CW compliance margin address. See DX-117 series.pdf (page 15) for a picture of what these mean
#define P_CCW_COMPLIANCE_MARGIN 0x1b            // CCW compliance margin address. The position change the motor can undergo before any torque is applied to counteract the torque
#define P_CW_COMPLIANCE_SLOPE  0x1c            // CW compliance slope address. This parameter requires further investigation
#define P_CCW_COMPLIANCE_SLOPE 0x1d            // CCW compliance slope address
#define P_PUNCH_L              0x30            // Punch lower byte address
#define P_PUNCH_H              0x31            // Punch higher byte address

// Motor Control
#define P_GOAL_POSITION_L      0x1e            // Goal position lower byte address
#define P_GOAL_POSITION_H      0x1f            // Goal position higher byte address. Use these addresses to control the position of the motor
#define P_GOAL_SPEED_L         0x20            // Goal speed lower byte address
#define P_GOAL_SPEED_H         0x21            // Goal speed higher byte address. Use these two addresses to control the speed at which the motor moves toward the goal position
#define P_TORQUE_LIMIT_L       0x22            // Limited torque lower byte address
#define P_TORQUE_LIMIT_H       0x23            // Limited torque higher byte address. These two bytes specify the torque at which the motor will shutdown (go limp) if desired

// Motor Feedback
#define P_PRESENT_POSITION_L   0x24            // Present position lower byte address
#define P_PRESENT_POSITION_H   0x25            // Present position higher byte address
#define P_PRESENT_SPEED_L      0x26            // Present speed lower byte address
#define P_PRESENT_SPEED_H      0x27            // Present speed higher byte address
#define P_PRESENT_LOAD_L       0x28            // Present load lower byte address
#define P_PRESENT_LOAD_H       0x29            // Present load higher byte address
#define P_PRESENT_VOLTAGE      0x2a            // Present voltage address
#define P_PRESENT_TEMPERATURE  0x2b            // Present temperature address

#define NUM_FEEDBACK_MOTOR     6               // The number of feedback bytes relating to the motor
#define NUM_FEEDBACK_ALL       8               // The number of feedback bytes

#endif
