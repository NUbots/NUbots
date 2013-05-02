/*
 *   CM730.cpp
 *
 *   Author: ROBOTIS
 *
 */
#include <stdio.h>
#include <cassert>
#include <sys/time.h>
#include <algorithm>

#include "FSR.h"
#include "CM730.h"
#include "MotionStatus.h"

using namespace Robot;


#define ID                  (2)
#define LENGTH              (3)
#define INSTRUCTION         (4)
#define ERRBIT              (4)
#define PARAMETER           (5)
#define DEFAULT_BAUDNUMBER  (1)

#define INST_PING       (1)
#define INST_READ       (2)
#define INST_WRITE      (3)
#define INST_REG_WRITE  (4)
#define INST_ACTION     (5)
#define INST_RESET      (6)
#define INST_SYNC_WRITE (131)   // 0x83
#define INST_BULK_READ  (146)   // 0x92


BulkReadData::BulkReadData() :
        start_address(0),
        length(0),
        error(-1)
{
    for(int i = 0; i < MX28::MAXNUM_ADDRESS; i++)
        table[i] = 0;
}

int BulkReadData::ReadByte(int address)
{
    if(address >= start_address &&
       address < (start_address + length))
        return (int)table[address];

    return 0;
}

int BulkReadData::ReadWord(int address)
{
    if(address >= start_address &&
       address < (start_address + length))
        return CM730::MakeWord(table[address], table[address+1]);

    return 0;
}


CM730::CM730(PlatformCM730 *platform)
{
    m_Platform = platform;
    DEBUG_PRINT = false;
    for(int i = 0; i < ID_BROADCAST; i++)
        bulk_read_data_[i] = BulkReadData();

    // Create the sensor read manager
    sensor_read_manager_ = new Robot::SensorReadManager();
}

CM730::~CM730()
{
    Disconnect();
}


void CM730::printInstructionType(unsigned char *txpacket)
{
    int instruction_value = txpacket[INSTRUCTION];
    fprintf(stderr, "INST: %s\n", getInstructionTypeString(instruction_value));
}

void CM730::PrintResultType(int error_code)
{
    fprintf(stderr, "RETURN: %s\n", getTxRxErrorString(error_code));
}

inline void CM730::PerformPriorityWait(int priority)
{
    if(priority > 1)
        m_Platform->LowPriorityWait();
    if(priority > 0)
        m_Platform->MidPriorityWait();
    m_Platform->HighPriorityWait();
}

inline void CM730::PerformPriorityRelease(int priority)
{
    m_Platform->HighPriorityRelease();
    if(priority > 0)
        m_Platform->MidPriorityRelease();
    if(priority > 1)
        m_Platform->LowPriorityRelease();
}


// Cm730 packet (communicate only to the CM730 controller board,
// and not attached devices)
// Note: This method is similar to CM730::TxRxBulkReadPacket.
//       See the comments there for help deciphering it.
//       (I'll refactor these soon. -MM)
inline void CM730::TxRxCMPacket(
    unsigned char* &txpacket,
    unsigned char* &rxpacket,
    int &res,
    int &length)
{
    int to_length = 0;

    if(txpacket[INSTRUCTION] == INST_READ)
        to_length = txpacket[PARAMETER+1] + 6;
    else
        to_length = 6;

    m_Platform->SetPacketTimeout(length);

    int get_length = 0;
    if(DEBUG_PRINT == true) fprintf(stderr, "RX: ");

    while(1)
    {
        length = m_Platform->ReadPort(&rxpacket[get_length], to_length - get_length);
        if(DEBUG_PRINT == true)
        {
            for(int n=0; n<length; n++)
                fprintf(stderr, "%.2X ", rxpacket[get_length + n]);
        }
        get_length += length;

        if(get_length == to_length)
        {
            // Find packet header
            int i;
            for(i = 0; i < (get_length - 1); i++)
            {
                if(rxpacket[i] == 0xFF && rxpacket[i+1] == 0xFF)
                    break;
                else if(i == (get_length - 2) && rxpacket[get_length - 1] == 0xFF)
                    break;
            }

            if(i == 0)
            {
                // Check checksum
                unsigned char checksum = CalculateChecksum(rxpacket);
                if(DEBUG_PRINT == true)
                    fprintf(stderr, "CHK:%.2X\n", checksum);

                if(rxpacket[get_length-1] == checksum)
                    res = SUCCESS;
                else
                    res = RX_CORRUPT;

                break;
            }
            else
            {
                for(int j = 0; j < (get_length - i); j++)
                    rxpacket[j] = rxpacket[j+i];
                get_length -= i;
            }
        }
        else
        {
            if(m_Platform->IsPacketTimeout() == true)
            {
                if(get_length == 0)
                    res = RX_TIMEOUT;
                else
                    res = RX_CORRUPT;

                break;
            }
        }
    }
}

double GetCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);

    return ((double)tv.tv_sec*1000.0 + (double)tv.tv_usec/1000.0);
}
double BulkReadTimer(bool reset = false)
{
    static double start_time = 0.0;
    double time;

    if(reset)
    {
        start_time = GetCurrentTime();
        return start_time;
    }

    time = GetCurrentTime() - start_time;
    if(time < 0.0)
        start_time = GetCurrentTime();

    return time;
}

void PrintSuccessfulBulkReadPeriod(Robot::PlatformCM730* m_Platform)
{
    static const int len = 512;
    static double successful_bulk_read_times[len] = { 0, }; // DEBUG
    static double max_successful_bulk_read_time = 0;
    static double min_successful_bulk_read_time = 100000;
    static int index = 0;

    double new_time = BulkReadTimer();
    BulkReadTimer(true);

    fprintf(stderr, "Read Success! (time = %fms)\n", new_time);
    
    successful_bulk_read_times[index] = new_time;
    ++index;
    if(index >= len) index = 0;
    
    if(max_successful_bulk_read_time < new_time && new_time < 1000)
        max_successful_bulk_read_time = new_time;

    if(min_successful_bulk_read_time > new_time)
        min_successful_bulk_read_time = new_time;

    double avg = 0;
    for(int i = 0; i < len; i++) 
        avg += successful_bulk_read_times[i];
    avg /= (double)len;
    
    fprintf(stderr, "Average successful bulk read period = %fms\n", avg);
    fprintf(stderr, "Maximum successful bulk read period = %fms\n", max_successful_bulk_read_time);
    fprintf(stderr, "Minimum successful bulk read period = %fms\n", min_successful_bulk_read_time);
}

void PrintSuccessfulBulkReadTimes(Robot::PlatformCM730* m_Platform)
{
    static const int len = 512;
    static double successful_bulk_read_times[len] = { 0, }; // DEBUG
    static double max_successful_bulk_read_time = 0;
    static double min_successful_bulk_read_time = 100000;
    static int index = 0;

    double new_time = m_Platform->GetPacketTime();
    fprintf(stderr, "Read Success! (time = %fms)\n", new_time);
    
    successful_bulk_read_times[index] = new_time;
    ++index;
    if(index >= len) index = 0;
    
    if(max_successful_bulk_read_time < new_time)
        max_successful_bulk_read_time = new_time;

    if(min_successful_bulk_read_time > new_time)
        min_successful_bulk_read_time = new_time;

    double avg = 0;
    for(int i = 0; i < len; i++) 
        avg += successful_bulk_read_times[i];
    avg /= (double)len;
    
    fprintf(stderr, "Average successful bulk read time = %fms\n", avg);
    fprintf(stderr, "Maximum successful bulk read time = %fms\n", max_successful_bulk_read_time);
    fprintf(stderr, "Minimum successful bulk read time = %fms\n", min_successful_bulk_read_time);
}

inline int CM730::ReceiveBulkReadResponseFromPort(
    unsigned char* rxpacket,
    int to_length,
    PlatformCM730 *m_Platform,
    int &res)
{
    if(DEBUG_PRINT == true) fprintf(stderr, "RX: ");

    // original multiplier of 1.5 appears to be an undocumented hack.
    m_Platform->SetPacketTimeout(to_length * 1.5 * 8);

    int get_length = 0; //! length in bytes of data read so far
    // Read data from port
    // (loop until all data has been read, or a timeout occurs)
    while(true)
    {
        // read some (more) data
        int length = m_Platform->ReadPort(&rxpacket[get_length], to_length - get_length);
        if(DEBUG_PRINT == true)
        {
            for(int n = 0; n < length; n++)
                fprintf(stderr, "%.2X ", rxpacket[get_length + n]);
        }
        get_length += length;

        // break if all data has been read
        if(get_length == to_length)
        {
            res = SUCCESS;
            PrintSuccessfulBulkReadTimes(m_Platform);
            PrintSuccessfulBulkReadPeriod(m_Platform);
            break;
        }

        // break, and set error code, if a timeout occured
        if(m_Platform->IsPacketTimeout() == true)
        {
            if(get_length == 0)
            {
                res = RX_TIMEOUT;
                fprintf(stderr, "RX_TIMEOUT: Reading data (time = %fms)\n", m_Platform->GetPacketTime());
            }
            else
            {
                res = RX_CORRUPT;
                fprintf(stderr, "RX_CORRUPT: Reading data (time = %fms)\n", m_Platform->GetPacketTime());
            }
            break;
        }
    }

    return get_length;
}

int CM730::AdvanceBuffer(unsigned char* buffer, int buffer_length,
                          int num_bytes_to_advance)
{
    int new_length = buffer_length - num_bytes_to_advance;
    
    for(int i = 0; i < new_length; i++)
        buffer[i] = buffer[i + num_bytes_to_advance];

    return new_length;
}

inline void CM730::TxRxBulkReadPacket(
    unsigned char* &txpacket,
    unsigned char* &rxpacket,
    int &res)
{
    int to_length = 0; //! length in bytes of data to read
    int num = (txpacket[LENGTH]-3) / 3; // number of blocks to read

    // Set bulkreaddata lengths and start addresses
    for(int x = 0; x < num; x++)
    {
        int _id   = txpacket[PARAMETER+(3*x)+2];
        int _len  = txpacket[PARAMETER+(3*x)+1];
        int _addr = txpacket[PARAMETER+(3*x)+3];

        to_length += _len + 6;
        bulk_read_data_[_id].length = _len;
        bulk_read_data_[_id].start_address = _addr;
    }

    // Read data from port
    int get_length = ReceiveBulkReadResponseFromPort(rxpacket, to_length,
                                                     m_Platform, res);
    

    // ResetBulkReadDataErrorCodes
    for(int x = 0; x < num; x++)
    {
        int _id = txpacket[PARAMETER+(3*x)+2];
        bulk_read_data_[_id].error = -1;
    }


    // Validate received data (in rxpacket) and
    // copy it into BulkReadData objects (in bulk_read_data_).
    while(1)
    {
        // Note: rxpacket may contain several sets of values to be read
        //       (e.g. one set for each motor, containing all the values from
        //        that motor's memory table).
        //
        // The first bytes of a valid set of values are the following:
        // 0x00: 0xFF      (literal value 0xFF)
        // 0x01: 0xFF      (literal value 0xFF)
        // 0x02: ID        (a byte indicating which motor/device's data follows)
        // 0x03: LENGTH    (the number of bytes following the error bit)
        // 0x04: ERRBIT    (stores an error code for this read)
        // 0x05: (PARAMETER) (the first byte of data from the device)
        //
        // ... (all the data)
        //
        // 0x03+(LENGTH): A checksum for this data packet.
        //                (Note: this is the last address of the packet)
        //
        // In particular, sets always start with the bytes, 0xFF 0xFF.
        //

        int i;
        for(i = 0; i < get_length - 1; i++)
        {
            if(rxpacket[i] == 0xFF && rxpacket[i+1] == 0xFF)
                break;
            else if(i == (get_length - 2) && rxpacket[get_length - 1] == 0xFF)
                break;
        }

        if(i != 0)
        {
            // skip invalid/header bytes
            get_length = AdvanceBuffer(rxpacket, get_length, i);

            // continue;
        }
        else
        // if(i == 0)
        {
            // Check checksum
            unsigned char checksum = CalculateChecksum(rxpacket);
            if(DEBUG_PRINT == true)
                fprintf(stderr, "CHK:%.2X\n", checksum);

            // If the last element is equal to the checksum...
            // Note: rxpacket[LENGTH] stores the number of bytes in rxpacket
            //       after the array position LENGTH.
            if(rxpacket[LENGTH+rxpacket[LENGTH]] == checksum)
            {
                // Copy data of first value to the datatable of the
                // corresponding BulkReadData object.
                int sensor_id = rxpacket[ID];
                BulkReadData& sensor_data = bulk_read_data_[sensor_id];

                for(int j = 0; j < (rxpacket[LENGTH]-2); j++)
                    sensor_data.table[sensor_data.start_address + j] = rxpacket[PARAMETER + j];

                sensor_data.error = (int)rxpacket[ERRBIT];

                // skip the rxpacket entry that was just read
                int cur_packet_length = LENGTH + 1 + rxpacket[LENGTH];
                to_length = AdvanceBuffer(rxpacket, get_length, cur_packet_length);
                get_length = to_length;

                num--;
            }
            else
            {
                fprintf(stderr, "RX_CORRUPT: Checksum.\n");
                res = RX_CORRUPT;
                
                // skip next 2 bytes of rxpacket
                to_length = AdvanceBuffer(rxpacket, get_length, 2);
                get_length = to_length;
            }

            if(num == 0) // rxpacket has been read entirely
                break;
            else if(get_length <= 6) // no room for any more data
            {
                // if(num != 0) // redundant
                // {
                    res = RX_CORRUPT;
                    fprintf(stderr, "RX_CORRUPT: Unexpected end of packet.\n");
                // }
                break;
            }

            // continue;
        }
    }
}

int CM730::TxRxPacket(unsigned char *txpacket, unsigned char *rxpacket, int priority)
{
    // Acquire resources
    PerformPriorityWait(priority);

    // m_Platform->Sleep(100); // DEBUG (crashes robot...)

    int res = TX_FAIL;
    int length = txpacket[LENGTH] + 4;

    txpacket[0] = 0xFF;
    txpacket[1] = 0xFF;
    txpacket[length - 1] = CalculateChecksum(txpacket);

    if(DEBUG_PRINT == true)
    {
        fprintf(stderr, "\nTX: ");
        for(int n=0; n<length; n++)
            fprintf(stderr, "%.2X ", txpacket[n]);

        printInstructionType(txpacket);
    }

    if(length < (MAXNUM_TXPARAM + 6)) // Enforce hardware/api limit on length of data to send.
    {
        // Transmit the packet:
        m_Platform->ClearPort();
        if(m_Platform->WritePort(txpacket, length) == length)
        {
            // Receive the response:

            if (txpacket[ID] != ID_BROADCAST) // i.e. Must be ID_CM.
            {
                TxRxCMPacket(txpacket, rxpacket, res, length); // Note: 'length' can be passed by value here.
            }
            else if(txpacket[INSTRUCTION] == INST_BULK_READ) // (INST_BULK_READ is an ID_BROADCAST)
            {
                TxRxBulkReadPacket(txpacket, rxpacket, res);
            }
            else
            {
                // i.e. Must be an ID_BROADCAST, and one of:
                //   - INST_PING
                //   - INST_READ
                //   - INST_WRITE
                //   - INST_REG_WRITE
                //   - INST_ACTION
                //   - INST_RESET
                //   - INST_SYNC_WRITE
                res = SUCCESS;
            }
        }
        else res = TX_FAIL;
    }
    else res = TX_CORRUPT;


    if(DEBUG_PRINT == true)
    {
        fprintf(stderr, "Time:%.2fms  ", m_Platform->GetPacketTime());
        PrintResultType(res);
    }

    // Release resources
    PerformPriorityRelease(priority);

    return res;
}

unsigned char CM730::CalculateChecksum(unsigned char *packet)
{
    unsigned char checksum = 0x00;
    for(int i=2; i<packet[LENGTH]+3; i++ )
        checksum += packet[i];
    return (~checksum);
}

int CM730::BulkRead()
{
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };

    // Note: This can be skipped if no errors have occured, and all sensors are
    //       responding for appropriately many consecutive reads.
    sensor_read_manager_->MakeBulkReadPacket(bulk_read_tx_packet_);

    // Perform the read operation from the CM730.
    // Note: Possible error codes are:
    //  { SUCCESS, TX_CORRUPT, TX_FAIL, RX_FAIL, RX_TIMEOUT, RX_CORRUPT }
    int bulk_read_error_code = TxRxPacket(bulk_read_tx_packet_, rxpacket, 0);
    
    bool error_occurred = sensor_read_manager_->ProcessBulkReadErrors(bulk_read_error_code, bulk_read_data_);

    return error_occurred;
}

int CM730::SyncWrite(int start_addr, int each_length, int number, int *pParam)
{
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
    int n;

    txpacket[ID]                = (unsigned char)ID_BROADCAST;
    txpacket[INSTRUCTION]       = INST_SYNC_WRITE;
    txpacket[PARAMETER]            = (unsigned char)start_addr;
    txpacket[PARAMETER + 1]        = (unsigned char)(each_length - 1);
    for(n = 0; n < (number * each_length); n++)
        txpacket[PARAMETER + 2 + n]   = (unsigned char)pParam[n];
    txpacket[LENGTH]            = n + 4;

    return TxRxPacket(txpacket, rxpacket, 0);
}

bool CM730::Connect()
{
    if(m_Platform->OpenPort() == false)
    {
        fprintf(stderr, "\n Fail to open port\n");
        fprintf(stderr, " CM-730 is used by another program or do not have root privileges.\n\n");
        return false;
    }

    return DXLPowerOn();
}

bool CM730::ChangeBaud(int baud)
{
    if(m_Platform->SetBaud(baud) == false)
    {
        fprintf(stderr, "\n Fail to change baudrate\n");
        return false;
    }

    return DXLPowerOn();
}

bool CM730::DXLPowerOn()
{
    if(WriteByte(CM730::ID_CM, CM730::P_DXL_POWER, 1, 0) == CM730::SUCCESS)
    {
        if(DEBUG_PRINT == true)
            fprintf(stderr, " Succeed to change Dynamixel power!\n");

        WriteWord(CM730::ID_CM, CM730::P_LED_HEAD_L, MakeColor(255, 128, 0), 0);
        m_Platform->Sleep(300); // about 300msec
    }
    else
    {
        if(DEBUG_PRINT == true)
            fprintf(stderr, " Fail to change Dynamixel power!\n");
        return false;
    }

    return true;
}

void CM730::Disconnect()
{
    // Make the Head LED to green
    //WriteWord(CM730::ID_CM, CM730::P_LED_HEAD_L, MakeColor(0, 255, 0), 0);
    unsigned char txpacket[] = {0xFF, 0xFF, 0xC8, 0x05, 0x03, 0x1A, 0xE0, 0x03, 0x32};
    m_Platform->WritePort(txpacket, 9);

    m_Platform->ClosePort();
}

int CM730::WriteByte(int address, int value, int *error)
{
    return WriteByte(ID_CM, address, value, error);
}

int CM730::WriteWord(int address, int value, int *error)
{
    return WriteWord(ID_CM, address, value, error);
}

int CM730::Ping(int id, int *error)
{
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
    int result;

    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = INST_PING;
    txpacket[LENGTH]       = 2;

    result = TxRxPacket(txpacket, rxpacket, 2);
    if(result == SUCCESS && txpacket[ID] != ID_BROADCAST)
    {
        if(error != 0)
            *error = (int)rxpacket[ERRBIT];
    }

    return result;
}

int CM730::ReadByte(int id, int address, int *pValue, int *error)
{
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
    int result;

    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = INST_READ;
    txpacket[PARAMETER]    = (unsigned char)address;
    txpacket[PARAMETER+1]  = 1;
    txpacket[LENGTH]       = 4;

    result = TxRxPacket(txpacket, rxpacket, 2);
    if(result == SUCCESS)
    {
        *pValue = (int)rxpacket[PARAMETER];
        if(error != 0)
            *error = (int)rxpacket[ERRBIT];
    }

    return result;
}

int CM730::ReadWord(int id, int address, int *pValue, int *error)
{
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
    int result;

    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = INST_READ;
    txpacket[PARAMETER]    = (unsigned char)address;
    txpacket[PARAMETER+1]  = 2;
    txpacket[LENGTH]       = 4;

    result = TxRxPacket(txpacket, rxpacket, 2);
    if(result == SUCCESS)
    {
        *pValue = MakeWord((int)rxpacket[PARAMETER], (int)rxpacket[PARAMETER + 1]);

        if(error != 0)
            *error = (int)rxpacket[ERRBIT];
    }

    return result;
}

int CM730::ReadTable(int id, int start_addr, int end_addr, unsigned char *table, int *error)
{
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
    int result;
    int length = end_addr - start_addr + 1;

    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = INST_READ;
    txpacket[PARAMETER]    = (unsigned char)start_addr;
    txpacket[PARAMETER+1]  = (unsigned char)length;
    txpacket[LENGTH]       = 4;

    result = TxRxPacket(txpacket, rxpacket, 1);
    if(result == SUCCESS)
    {
        for(int i=0; i<length; i++)
            table[start_addr + i] = rxpacket[PARAMETER + i];

        if(error != 0)
            *error = (int)rxpacket[ERRBIT];
    }

    return result;
}

int CM730::WriteByte(int id, int address, int value, int *error)
{
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
    int result;

    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = INST_WRITE;
    txpacket[PARAMETER]    = (unsigned char)address;
    txpacket[PARAMETER+1]  = (unsigned char)value;
    txpacket[LENGTH]       = 4;

    result = TxRxPacket(txpacket, rxpacket, 2);
    if(result == SUCCESS && id != ID_BROADCAST)
    {
        if(error != 0)
            *error = (int)rxpacket[ERRBIT];
    }

    return result;
}

int CM730::WriteWord(int id, int address, int value, int *error)
{
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
    int result;

    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = INST_WRITE;
    txpacket[PARAMETER]    = (unsigned char)address;
    txpacket[PARAMETER+1]  = (unsigned char)GetLowByte(value);
    txpacket[PARAMETER+2]  = (unsigned char)GetHighByte(value);
    txpacket[LENGTH]       = 5;

    result = TxRxPacket(txpacket, rxpacket, 2);
    if(result == SUCCESS && id != ID_BROADCAST)
    {
        if(error != 0)
            *error = (int)rxpacket[ERRBIT];
    }

    return result;
}

int CM730::MakeWord(int lowbyte, int highbyte)
{
    unsigned short word;

    word = highbyte;
    word = word << 8;
    word = word + lowbyte;

    return (int)word;
}

int CM730::GetLowByte(int word)
{
    unsigned short temp;
    temp = word & 0xff;
    return (int)temp;
}

int CM730::GetHighByte(int word)
{
    unsigned short temp;
    temp = word & 0xff00;
    return (int)(temp >> 8);
}

int CM730::MakeColor(int red, int green, int blue)
{
    int r = red & 0xFF;
    int g = green & 0xFF;
    int b = blue & 0xFF;

    return (int)(((b>>3)<<10)|((g>>3)<<5)|(r>>3));
}

const char* CM730::getTxRxErrorString(int error_code)
{
    switch(error_code)
    {
    case SUCCESS   : return "SUCCESS"   ;
    case TX_CORRUPT: return "TX_CORRUPT";
    case TX_FAIL   : return "TX_FAIL"   ;
    case RX_FAIL   : return "RX_FAIL"   ;
    case RX_TIMEOUT: return "RX_TIMEOUT";
    case RX_CORRUPT: return "RX_CORRUPT";
    default        : return "UNKNOWN"   ;
    }
}

const char* CM730::getInstructionTypeString(int instruction_value)
{
    switch(instruction_value)
    {
    case INST_PING      : return "PING"      ;
    case INST_READ      : return "READ"      ;
    case INST_WRITE     : return "WRITE"     ;
    case INST_REG_WRITE : return "REG_WRITE" ;
    case INST_ACTION    : return "ACTION"    ;
    case INST_RESET     : return "RESET"     ;
    case INST_SYNC_WRITE: return "SYNC_WRITE";
    case INST_BULK_READ : return "BULK_READ" ;
    default             : return "UNKNOWN"   ;
    }
}
