#include "FirmwareInstaller.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "extension/Configuration.h"

#include "message/platform/darwin/Firmware.h"

namespace module {
namespace tools {

    using extension::Configuration;

    using message::platform::darwin::FlashCM730Firmware;

    FirmwareInstaller::FirmwareInstaller(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , device("UNKNOWN")
        , flashCM730(false)
        , flashDynamixel(false)
        , cm730Firmware("INVALID")
        , dynamixelFirmware("INVALID") {

        on<Configuration>("FirmwareInstaller.yaml").then([this](const Configuration& config) {
            device            = config["device"].as<std::string>();
            flashCM730        = config["CM730"]["flash"].as<bool>();
            cm730Firmware     = config["CM730"]["firmware"].as<std::string>();
            flashDynamixel    = config["dynamixel"]["flash"].as<bool>();
            dynamixelFirmware = config["dynamixel"]["firmware"].as<std::string>();
        });

        on<Startup>().then("Firmware Installer Startup", [this] {
            if (flashCM730) {
                std::vector<uint8_t> firmware(MEMORY_MAXSIZE, 0xFF);
                uint32_t startAddress = 0, binSize = 0;

                if (!hex2bin(cm730Firmware, firmware.data(), startAddress, binSize)) {
                    log<NUClear::FATAL>("Failed to convert CM730 firmware to binary format.");
                }

                else {
                    log<NUClear::FATAL>("CM730 firmware converted to binary format. Commencing flashing procedure...");
                    emit(std::make_unique<FlashCM730Firmware>(FlashCM730Firmware{firmware, startAddress, binSize}));
                }
            }

            if (flashDynamixel) {
                log<NUClear::FATAL>("Flashing Dynamixel firmware is not implemented yet.");
            }
        });
    }

    bool FirmwareInstaller::hex2bin(const std::string& hexFile,
                                    uint8_t* pBinBuffer,
                                    uint32_t& startAddress,
                                    uint32_t& bufSize) {
        std::string line, data;
        uint32_t numBytes, type;
        uint32_t firstWord, address, segment;
        uint32_t lowestAddress, highestAddress, physicalAddress;
        uint16_t temp;
        int i, iIndex;
        bool bRead = true;

        std::ifstream ifs(hexFile);

        if (!ifs.is_open()) {
            log<NUClear::FATAL>("Unable to open firmware file:", hexFile);
            return false;
        }

        segment        = 0x00;
        lowestAddress  = MEMORY_MAXSIZE - 1;
        highestAddress = 0x00;

        do {
            // Read a line from input file.
            std::getline(ifs, line);

            if ((!ifs) || (line.length() == 0)) {
                log<NUClear::FATAL>(hexFile, "has invalid format.");
                ifs.close();
                return false;
            }

            /* Scan the first two bytes and nb of bytes.
            The two bytes are read in firstWord since it's use depend on the
            record type: if it's an extended address record or a data record.
            */
            if (line.front() != ':') {
                log<NUClear::FATAL>(hexFile, "has invalid format.");
                ifs.close();
                return false;
            }

            line.erase(std::remove_if(line.begin(), line.end(), [](char c) -> bool { return std::isspace(c); }),
                       line.end());

            std::stringstream ss(line);
            ss.ignore(1, ':');
            ss >> std::hex >> std::setw(2) >> numBytes;
            ss >> std::hex >> std::setw(4) >> firstWord;
            ss >> std::hex >> std::setw(2) >> type;
            ss >> data;

            /* If we're reading the last record, ignore it. */
            switch (type) {
                /* Data record */
                case 0: {
                    address         = firstWord;
                    physicalAddress = ((segment << 4) & ADDRESS_MASK) + address;

                    /* Check that the physical address stays in the buffer's range. */
                    if ((physicalAddress + numBytes) <= (MEMORY_MAXSIZE - 1)) {
                        /* Set the lowest address as base pointer. */
                        if (lowestAddress > physicalAddress) {
                            lowestAddress = physicalAddress;
                        }

                        /* Same for the top address. */
                        if (highestAddress < (physicalAddress + numBytes - 1)) {
                            highestAddress = (physicalAddress + numBytes - 1);
                        }

                        /* Read the Data bytes. */
                        /* Bytes are written in the Memory block even if checksum is wrong. */
                        for (i = numBytes, iIndex = 0; i > 0; i--, iIndex += 2) {
                            std::stringstream ss(data.substr(iIndex));
                            ss >> std::hex >> std::setw(2) >> temp;

                            pBinBuffer[physicalAddress] = (unsigned char) temp;
                            physicalAddress++;
                        }

                        /* Read the Checksum value. */
                        std::stringstream ss(data.substr(iIndex));
                        ss >> std::hex >> std::setw(2) >> temp;
                    }

                    else {
                        log<NUClear::FATAL>("Exceed maximum memory size!");
                        ifs.close();
                        return false;
                    }

                    break;
                }

                /* End of file record */
                case 1: {
                    /* Simply ignore checksum errors in this line. */
                    bRead = false;
                    break;
                }

                /* Extended segment address record */
                case 2: {
                    /* firstWord contains the offset. It's supposed to be 0000 so
                    we ignore it. */
                    std::stringstream ss(data);
                    ss >> std::hex >> std::setw(4) >> segment;
                    ss >> std::hex >> std::setw(2) >> temp;

                    /* Update the current address. */
                    physicalAddress = (segment << 4) & ADDRESS_MASK;
                    break;
                }

                /* Start segment address record */
                case 3: {
                    /* Nothing to be done since it's for specifying the starting address for
                    execution of the binary code */
                    break;
                }

                /* Extended linear address record */
                case 4: {
                    break;
                }

                /* Start linear address record */
                case 5: {
                    /* Nothing to be done since it's for specifying the starting address for
                    execution of the binary code */
                    break;
                }

                default: {
                    log<NUClear::FATAL>("Can not support type:", type);
                    ifs.close();
                    return false;
                }
            }

        } while (bRead == true);

        bufSize      = highestAddress - lowestAddress + 1;
        startAddress = lowestAddress;

        ifs.close();
        return true;
    }

    /*
    *** For Dynamixel flashing. ***
    void FirmwareInstaller::Reset(CM730 *cm730, int id) {
        int FailCount = 0;
        int FailMaxCount = 10;
        printf(" Reset ID:%d...", id);

        if(cm730->Ping(id, 0) != CM730::SUCCESS)
        {
            printf("Fail\n");
            return;
        }

        FailCount = 0;
        while(1)
        {
            if(cm730->WriteByte(id, MX28::P_RETURN_DELAY_TIME, 0, 0) == CM730::SUCCESS)
                break;

            FailCount++;
            if(FailCount > FailMaxCount)
            {
                printf("Fail\n");
                return;
            }
            usleep(50000);
        }

        FailCount = 0;
        while(1)
        {
            if(cm730->WriteByte(id, MX28::P_RETURN_LEVEL, 2, 0) == CM730::SUCCESS)
                break;

            FailCount++;
            if(FailCount > FailMaxCount)
            {
                printf("Fail\n");
                return;
            }
            usleep(50000);
        }

        if(id != CM730::ID_CM)
        {
            double cwLimit = MX28::MIN_ANGLE;
            double ccwLimit = MX28::MAX_ANGLE;

            switch(id)
            {
            case JointData::ID_R_SHOULDER_ROLL:
                cwLimit = -75.0;
                ccwLimit = 135.0;
                break;

            case JointData::ID_L_SHOULDER_ROLL:
                cwLimit = -135.0;
                ccwLimit = 75.0;
                break;

            case JointData::ID_R_ELBOW:
                cwLimit = -95.0;
                ccwLimit = 70.0;
                break;

            case JointData::ID_L_ELBOW:
                cwLimit = -70.0;
                ccwLimit = 95.0;
                break;

            case JointData::ID_R_HIP_YAW:
                cwLimit = -123.0;
                ccwLimit = 53.0;
                break;

            case JointData::ID_L_HIP_YAW:
                cwLimit = -53.0;
                ccwLimit = 123.0;
                break;

            case JointData::ID_R_HIP_ROLL:
                cwLimit = -45.0;
                ccwLimit = 59.0;
                break;

            case JointData::ID_L_HIP_ROLL:
                cwLimit = -59.0;
                ccwLimit = 45.0;
                break;

            case JointData::ID_R_HIP_PITCH:
                cwLimit = -100.0;
                ccwLimit = 29.0;
                break;

            case JointData::ID_L_HIP_PITCH:
                cwLimit = -29.0;
                ccwLimit = 100.0;
                break;

            case JointData::ID_R_KNEE:
                cwLimit = -6.0;
                ccwLimit = 130.0;
                break;

            case JointData::ID_L_KNEE:
                cwLimit = -130.0;
                ccwLimit = 6.0;
                break;

            case JointData::ID_R_ANKLE_PITCH:
                cwLimit = -72.0;
                ccwLimit = 80.0;
                break;

            case JointData::ID_L_ANKLE_PITCH:
                cwLimit = -80.0;
                ccwLimit = 72.0;
                break;

            case JointData::ID_R_ANKLE_ROLL:
                cwLimit = -44.0;
                ccwLimit = 63.0;
                break;

            case JointData::ID_L_ANKLE_ROLL:
                cwLimit = -63.0;
                ccwLimit = 44.0;
                break;

            case JointData::ID_HEAD_TILT:
                cwLimit = -25.0;
                ccwLimit = 55.0;
                break;
            }

            FailCount = 0;
            while(1)
            {
                if(cm730->WriteWord(id, MX28::P_CW_ANGLE_LIMIT_L, MX28::Angle2Value(cwLimit), 0) == CM730::SUCCESS)
                    break;

                FailCount++;
                if(FailCount > FailMaxCount)
                {
                    printf("Fail\n");
                    return;
                }
                usleep(50000);
            }
            FailCount = 0;
            while(1)
            {
                if(cm730->WriteWord(id, MX28::P_CCW_ANGLE_LIMIT_L, MX28::Angle2Value(ccwLimit), 0) == CM730::SUCCESS)
                    break;

                FailCount++;
                if(FailCount > FailMaxCount)
                {
                    printf("Fail\n");
                    return;
                }
                usleep(50000);
            }
            FailCount = 0;
            while(1)
            {
                if(cm730->WriteByte(id, MX28::P_HIGH_LIMIT_TEMPERATURE, 80, 0) == CM730::SUCCESS)
                    break;

                FailCount++;
                if(FailCount > FailMaxCount)
                {
                    printf("Fail\n");
                    return;
                }
                usleep(50000);
            }
            FailCount = 0;
            while(1)
            {
                if(cm730->WriteByte(id, MX28::P_LOW_LIMIT_VOLTAGE, 60, 0) == CM730::SUCCESS)
                    break;

                FailCount++;
                if(FailCount > FailMaxCount)
                {
                    printf("Fail\n");
                    return;
                }
                usleep(50000);
            }
            FailCount = 0;
            while(1)
            {
                if(cm730->WriteByte(id, MX28::P_HIGH_LIMIT_VOLTAGE, 140, 0) == CM730::SUCCESS)
                    break;

                FailCount++;
                if(FailCount > FailMaxCount)
                {
                    printf("Fail\n");
                    return;
                }
                usleep(50000);
            }
            FailCount = 0;
            while(1)
            {
                if(cm730->WriteWord(id, MX28::P_MAX_TORQUE_L, MX28::MAX_VALUE, 0) == CM730::SUCCESS)
                    break;

                FailCount++;
                if(FailCount > FailMaxCount)
                {
                    printf("Fail\n");
                    return;
                }
                usleep(50000);
            }
            FailCount = 0;
            while(1)
            {
                if(cm730->WriteByte(id, MX28::P_ALARM_LED, 36, 0) == CM730::SUCCESS) // Overload, Overheat
                    break;

                FailCount++;
                if(FailCount > FailMaxCount)
                {
                    printf("Fail\n");
                    return;
                }
                usleep(50000);
            }
            FailCount = 0;
            while(1)
            {
                if(cm730->WriteByte(id, MX28::P_ALARM_SHUTDOWN, 36, 0) == CM730::SUCCESS) // Overload, Overheat
                    break;

                FailCount++;
                if(FailCount > FailMaxCount)
                {
                    printf("Fail\n");
                    return;
                }
                usleep(50000);
            }
        }

        printf("Success\n");
    }
    */
}
}
