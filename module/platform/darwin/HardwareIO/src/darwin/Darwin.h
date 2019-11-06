/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef DARWIN_DARWIN_H
#define DARWIN_DARWIN_H

#include <cstdint>
#include <utility>
#include <vector>

#include "CM740.h"
#include "DarwinRawSensors.h"
#include "FSR.h"
#include "MX28.h"
#include "extension/Configuration.h"

namespace Darwin {

/**
 * Contains all of the dynamixel IDs in the system
 */
namespace ID {
    enum ID {
        CM740            = 200,
        R_SHOULDER_PITCH = 1,
        L_SHOULDER_PITCH = 2,
        R_SHOULDER_ROLL  = 3,
        L_SHOULDER_ROLL  = 4,
        R_ELBOW          = 5,
        L_ELBOW          = 6,
        R_HIP_YAW        = 7,
        L_HIP_YAW        = 8,
        R_HIP_ROLL       = 9,
        L_HIP_ROLL       = 10,
        R_HIP_PITCH      = 11,
        L_HIP_PITCH      = 12,
        R_KNEE           = 13,
        L_KNEE           = 14,
        R_ANKLE_PITCH    = 15,
        L_ANKLE_PITCH    = 16,
        R_ANKLE_ROLL     = 17,
        L_ANKLE_ROLL     = 18,
        HEAD_YAW         = 19,
        HEAD_PITCH       = 20,
        R_FSR            = 111,
        L_FSR            = 112,
        BROADCAST        = 254
    };
}  // namespace ID

/**
 * @brief The main class that others will use to interact with the CM740 and attached devices.
 *
 * @details
 *  This is the main access point for all users of this CM740 driver. Note that it is build for a little endian
 *  system, and if it is used on a big endian system, the code will need to be reviewed. This is because it is
 *  reading the 2 byte values as they are on the CM740 (which is little endian).
 *
 * @author Trent Houliston
 */
class Darwin {
private:
    /// Our UART class that we will communicate through
    UART uart;
    /// Which servos we should be building for
    std::vector<uint8_t> enabledServoIds;

    /// Our Prebuilt bulk read command
    std::vector<uint8_t> bulkReadCommand;

    /**
     * @brief Builds a bulk read packet to read all of the sensors.
     */
    void buildBulkReadPacket();

public:
    void setConfig(const extension::Configuration& config);

    /// The CM740
    CM740 cm740;
    /// The Right Shoulder Pitch MX28
    MX28 rShoulderPitch;
    /// The Left Shoulder Pitch MX28
    MX28 lShoulderPitch;
    /// The Right Shoulder Roll MX28
    MX28 rShoulderRoll;
    /// The Left Shoulder Roll MX28
    MX28 lShoulderRoll;
    /// The Right Elbow MX28
    MX28 rElbow;
    /// The Left Elbow MX28
    MX28 lElbow;
    /// The Right Hip Yaw MX28
    MX28 rHipYaw;
    /// The Left Hip Yaw MX28
    MX28 lHipYaw;
    /// The Right Hip Roll MX28
    MX28 rHipRoll;
    /// The Left Hip Roll MX28
    MX28 lHipRoll;
    /// The Right Hip Pitch MX28
    MX28 rHipPitch;
    /// The Left Hip Pitch MX28
    MX28 lHipPitch;
    /// The Right Knee MX28
    MX28 rKnee;
    /// The Left Knee MX28
    MX28 lKnee;
    /// The Right Ankle Pitch MX28
    MX28 rAnklePitch;
    /// The Left Ankle Pitch MX28
    MX28 lAnklePitch;
    /// The Right Ankle Roll MX28
    MX28 rAnkleRoll;
    /// The Left Ankle Roll MX28
    MX28 lAnkleRoll;
    /// The Head Pan MX28
    MX28 headPan;
    /// The Head Tilt MX28
    MX28 headTilt;
    /// The Right Foot FSR
    FSR rFSR;
    /// The Left Foot FSR
    FSR lFSR;

    /**
     * @brief Gets the darwin device with the given sensor id
     *
     * @param id the ID of the device to get (e.g. 200 for the CM740)
     *
     * @return the DarwinDevice object that controls this id
     */
    DarwinDevice& operator[](int id);

    /**
     * @brief Constructs a new Darwin instance and sets up communication with the CM740.
     *
     * @param name the file handle for the device the CM740 is connected to (e.g. /dev/ttyUSB0)
     */
    explicit Darwin(const char* name);

    /**
     * @brief Pings all of the attached devices, and returns a map containing their ID and if they were contactable.
     *
     * @return a map containing IDs and if they were contactable (returned no error code)
     */
    std::vector<std::pair<uint8_t, bool>> selfTest();

    /**
     * @brief This reads all of the sensors in a predefined pattern of what is considered "Interesting"
     *
     * @return A BulkReadResuts object containing all of the sensor data as it was read from the device (no trasforms)
     */
    BulkReadResults bulkRead();

    /**
     * @brief This sends a raw command to the UART that the dynamixels are on without expecting a response
     */
    void sendRawCommand(std::vector<uint8_t>& packet);
};
}  // namespace Darwin

#endif
