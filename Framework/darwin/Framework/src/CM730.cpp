/*
 *   CM730.cpp
 *
 *   Author: ROBOTIS
 *
 */
#include <stdio.h>
#include "FSR.h"
#include "CM730.h"
#include "MotionStatus.h"

#include <cassert>

using namespace Robot;


#define ID					(2)
#define LENGTH				(3)
#define INSTRUCTION			(4)
#define ERRBIT				(4)
#define PARAMETER			(5)
#define DEFAULT_BAUDNUMBER	(1)

#define INST_PING			(1)
#define INST_READ			(2)
#define INST_WRITE			(3)
#define INST_REG_WRITE		(4)
#define INST_ACTION			(5)
#define INST_RESET			(6)
#define INST_SYNC_WRITE		(131)   // 0x83
#define INST_BULK_READ      (146)   // 0x92


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
	    m_BulkReadData[i] = BulkReadData();
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

void CM730::printResultType(int error_code)
{
    fprintf(stderr, "RETURN: %s\n", getTxRxErrorString(error_code));
}

inline void CM730::performPriorityWait(int priority)
{
    if(priority > 1)
        m_Platform->LowPriorityWait();
    if(priority > 0)
        m_Platform->MidPriorityWait();
    m_Platform->HighPriorityWait();
}

inline void CM730::performPriorityRelease(int priority)
{
    m_Platform->HighPriorityRelease();
    if(priority > 0)
        m_Platform->MidPriorityRelease();
    if(priority > 1)
        m_Platform->LowPriorityRelease();
}


// CoMmand packet? (simply checks whether the commands sent were successful)
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

inline void CM730::TxRxBulkReadPacket(
    unsigned char* &txpacket, 
    unsigned char* &rxpacket, 
    int &res, 
    int &length)
{
    int to_length = 0; //! length in bytes of data to read
    int num = (txpacket[LENGTH]-3) / 3;

    // Set bulkreaddata lengths and start addresses
    for(int x = 0; x < num; x++)
    {
        int _id = txpacket[PARAMETER+(3*x)+2];
        int _len = txpacket[PARAMETER+(3*x)+1];
        int _addr = txpacket[PARAMETER+(3*x)+3];

        to_length += _len + 6;
        m_BulkReadData[_id].length = _len;
        m_BulkReadData[_id].start_address = _addr;
    }

    m_Platform->SetPacketTimeout(to_length*1.5);

    if(DEBUG_PRINT == true) fprintf(stderr, "RX: ");


    int get_length = 0; //! length in bytes of data read so far
    
    // Read data from port 
    // (loop until all data has been read, or a timeout occurs)
    while(1)
    {
        // read some (more) data
        length = m_Platform->ReadPort(&rxpacket[get_length], to_length - get_length);
        if(DEBUG_PRINT == true)
        {
            for(int n=0; n<length; n++)
                fprintf(stderr, "%.2X ", rxpacket[get_length + n]);
        }
        get_length += length;

        // break if all data has been read
        if(get_length == to_length)
        {
            res = SUCCESS;
            break;
        }
        else
        {
            // break, and set error code, if a timeout occured
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

    // resetBulkReadDataErrorCodes
    for(int x = 0; x < num; x++)
    {
        int _id = txpacket[PARAMETER+(3*x)+2];
        m_BulkReadData[_id].error = -1;
    }


    // Validate received data (in rxpacket) and
    // copy it into BulkReadData objects (in m_BulkReadData).
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
        // (LENGTH)+0x03: A checksum for this data packet. 
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
            for(int j = 0; j < (get_length - i); j++)
                rxpacket[j] = rxpacket[j+i];
            get_length -= i;

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
                for(int j = 0; j < (rxpacket[LENGTH]-2); j++)
                    m_BulkReadData[rxpacket[ID]].table[m_BulkReadData[rxpacket[ID]].start_address + j] = rxpacket[PARAMETER + j];

                m_BulkReadData[rxpacket[ID]].error = (int)rxpacket[ERRBIT];

                // skip the rxpacket entry that was just read
                int cur_packet_length = LENGTH + 1 + rxpacket[LENGTH];
                to_length = get_length - cur_packet_length;
                for(int j = 0; j <= to_length; j++)
                    rxpacket[j] = rxpacket[j+cur_packet_length];

                get_length = to_length;
                num--;
            }
            else
            {
                res = RX_CORRUPT;

                // skip next 2 bytes of rxpacket
                for(int j = 0; j <= get_length - 2; j++)
                    rxpacket[j] = rxpacket[j+2];

                to_length = get_length -= 2;
            }

            if(num == 0)
                break;
            else if(get_length <= 6) // rxpacket has been read entirely
            {
                if(num != 0) res = RX_CORRUPT;
                break;
            }

            // continue;
        }
    }
}

// int CM730::TxRxPacket(unsigned char *txpacket, unsigned char *rxpacket, int priority)
// {
// 	// Acquire resources
//     performPriorityWait(priority);

//     int res = TX_FAIL;
//     int length = txpacket[LENGTH] + 4;

//     txpacket[0] = 0xFF;
//     txpacket[1] = 0xFF;
//     txpacket[length - 1] = CalculateChecksum(txpacket);

//     if(DEBUG_PRINT == true) 
//     {
//         fprintf(stderr, "\nTX: ");
//         for(int n=0; n<length; n++)
//             fprintf(stderr, "%.2X ", txpacket[n]);

//         printInstructionType(txpacket);
//     }
    
//     if(length < (MAXNUM_TXPARAM + 6)) // Enforce hardware/api limit on length of data to send.
//     {
//         // Transmit the packet:
//         m_Platform->ClearPort();
//         if(m_Platform->WritePort(txpacket, length) == length)
//         {
//             // Receive the response:
            
//             if (txpacket[ID] != ID_BROADCAST) // i.e. Must be ID_CM.
//             {
//                 TxRxCMPacket(txpacket, rxpacket, res, length); // Note: 'length' can be passed by value here.
//             }
//             else if(txpacket[INSTRUCTION] == INST_BULK_READ) // (INST_BULK_READ is an ID_BROADCAST)
//             {
//                 TxRxBulkReadPacket(txpacket, rxpacket, res, length); // Note: length input is unnecessary.
//             }
//             else
//             {
//                 // i.e. Must be an ID_BROADCAST, and one of:
//                 //   - INST_PING
//                 //   - INST_READ
//                 //   - INST_WRITE
//                 //   - INST_REG_WRITE
//                 //   - INST_ACTION
//                 //   - INST_RESET
//                 //   - INST_SYNC_WRITE
//                 res = SUCCESS;
//             }
//         }
//         else res = TX_FAIL;      
//     }
//     else res = TX_CORRUPT;


// 	if(DEBUG_PRINT == true) 
//     {
//         fprintf(stderr, "Time:%.2fms  ", m_Platform->GetPacketTime());
//         printResultType(res);
//     }
	
//     // Release resources
// 	performPriorityRelease(priority);

// 	return res;
// }

int CM730::TxRxPacket(unsigned char *txpacket, unsigned char *rxpacket, int priority)
{
    if(priority > 1)
        m_Platform->LowPriorityWait();
    if(priority > 0)
        m_Platform->MidPriorityWait();
    m_Platform->HighPriorityWait();

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

        fprintf(stderr, "INST: ");
        switch(txpacket[INSTRUCTION])
        {
        case INST_PING:
            fprintf(stderr, "PING\n");
            break;

        case INST_READ:
            fprintf(stderr, "READ\n");
            break;

        case INST_WRITE:
            fprintf(stderr, "WRITE\n");
            break;

        case INST_REG_WRITE:
            fprintf(stderr, "REG_WRITE\n");
            break;

        case INST_ACTION:
            fprintf(stderr, "ACTION\n");
            break;

        case INST_RESET:
            fprintf(stderr, "RESET\n");
            break;

        case INST_SYNC_WRITE:
            fprintf(stderr, "SYNC_WRITE\n");
            break;

        case INST_BULK_READ:
            fprintf(stderr, "BULK_READ\n");
            break;

        default:
            fprintf(stderr, "UNKNOWN\n");
            break;
        }
    }

    if(length < (MAXNUM_TXPARAM + 6))
    {
        m_Platform->ClearPort();
        if(m_Platform->WritePort(txpacket, length) == length)
        {
            if (txpacket[ID] != ID_BROADCAST)
            {
                int to_length = 0;

                if(txpacket[INSTRUCTION] == INST_READ)
                    to_length = txpacket[PARAMETER+1] + 6;
                else
                    to_length = 6;

                m_Platform->SetPacketTimeout(length);

                int get_length = 0;
                if(DEBUG_PRINT == true)
                    fprintf(stderr, "RX: ");

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
            else if(txpacket[INSTRUCTION] == INST_BULK_READ)
            {
                int to_length = 0;
                int num = (txpacket[LENGTH]-3) / 3;

                for(int x = 0; x < num; x++)
                {
                    int _id = txpacket[PARAMETER+(3*x)+2];
                    int _len = txpacket[PARAMETER+(3*x)+1];
                    int _addr = txpacket[PARAMETER+(3*x)+3];

                    to_length += _len + 6;
                    m_BulkReadData[_id].length = _len;
                    m_BulkReadData[_id].start_address = _addr;
                }

                m_Platform->SetPacketTimeout(to_length*1.5);

                int get_length = 0;
                if(DEBUG_PRINT == true)
                    fprintf(stderr, "RX: ");

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
                        res = SUCCESS;
                        break;
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

                for(int x = 0; x < num; x++)
                {
                    int _id = txpacket[PARAMETER+(3*x)+2];
                    m_BulkReadData[_id].error = -1;
                }

                while(1)
                {
                    int i;
                    for(i = 0; i < get_length - 1; i++)
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

                        if(rxpacket[LENGTH+rxpacket[LENGTH]] == checksum)
                        {
                            for(int j = 0; j < (rxpacket[LENGTH]-2); j++)
                                m_BulkReadData[rxpacket[ID]].table[m_BulkReadData[rxpacket[ID]].start_address + j] = rxpacket[PARAMETER + j];

                            m_BulkReadData[rxpacket[ID]].error = (int)rxpacket[ERRBIT];

                            int cur_packet_length = LENGTH + 1 + rxpacket[LENGTH];
                            to_length = get_length - cur_packet_length;
                            for(int j = 0; j <= to_length; j++)
                                rxpacket[j] = rxpacket[j+cur_packet_length];

                            get_length = to_length;
                            num--;
                        }
                        else
                        {
                            res = RX_CORRUPT;

                            for(int j = 0; j <= get_length - 2; j++)
                                rxpacket[j] = rxpacket[j+2];

                            to_length = get_length -= 2;
                        }

                        if(num == 0)
                            break;
                        else if(get_length <= 6)
                        {
                            if(num != 0) res = RX_CORRUPT;
                            break;
                        }

                    }
                    else
                    {
                        for(int j = 0; j < (get_length - i); j++)
                            rxpacket[j] = rxpacket[j+i];
                        get_length -= i;
                    }
                }
            }
            else
                res = SUCCESS;          
        }
        else
            res = TX_FAIL;      
    }
    else
        res = TX_CORRUPT;

    if(DEBUG_PRINT == true)
    {
        fprintf(stderr, "Time:%.2fms  ", m_Platform->GetPacketTime());
        fprintf(stderr, "RETURN: ");
        switch(res)
        {
        case SUCCESS:
            fprintf(stderr, "SUCCESS\n");
            break;

        case TX_CORRUPT:
            fprintf(stderr, "TX_CORRUPT\n");
            break;

        case TX_FAIL:
            fprintf(stderr, "TX_FAIL\n");
            break;

        case RX_FAIL:
            fprintf(stderr, "RX_FAIL\n");
            break;

        case RX_TIMEOUT:
            fprintf(stderr, "RX_TIMEOUT\n");
            break;

        case RX_CORRUPT:
            fprintf(stderr, "RX_CORRUPT\n");
            break;

        default:
            fprintf(stderr, "UNKNOWN\n");
            break;
        }
    }

    m_Platform->HighPriorityRelease();
    if(priority > 0)
        m_Platform->MidPriorityRelease();
    if(priority > 1)
        m_Platform->LowPriorityRelease();

    return res;
}

unsigned char CM730::CalculateChecksum(unsigned char *packet)
{
	unsigned char checksum = 0x00;
	for(int i=2; i<packet[LENGTH]+3; i++ )
		checksum += packet[i];
	return (~checksum);
}

void CM730::MakeBulkReadPacket()
{
    int number = 0;

    m_BulkReadTxPacket[ID]              = (unsigned char)ID_BROADCAST;
    m_BulkReadTxPacket[INSTRUCTION]     = INST_BULK_READ;
    m_BulkReadTxPacket[PARAMETER]       = (unsigned char)0x0;

    if(Ping(CM730::ID_CM, 0) == SUCCESS)
    {
        m_BulkReadTxPacket[PARAMETER+3*number+1] = 20;
        m_BulkReadTxPacket[PARAMETER+3*number+2] = CM730::ID_CM;
        m_BulkReadTxPacket[PARAMETER+3*number+3] = CM730::P_BUTTON;
        number++;
    }

    for(int id = 1; id < JointData::NUMBER_OF_JOINTS; id++)
    {
//        if(MotionStatus::m_CurrentJoints.GetEnable(id))
//        {
            m_BulkReadTxPacket[PARAMETER+3*number+1] = 2;   // length
            m_BulkReadTxPacket[PARAMETER+3*number+2] = id;  // id
            m_BulkReadTxPacket[PARAMETER+3*number+3] = MX28::P_PRESENT_POSITION_L; // start address
            number++;
//        }
    }

    if(Ping(FSR::ID_L_FSR, 0) == SUCCESS)
    {
        m_BulkReadTxPacket[PARAMETER+3*number+1] = 10;               // length
        m_BulkReadTxPacket[PARAMETER+3*number+2] = FSR::ID_L_FSR;   // id
        m_BulkReadTxPacket[PARAMETER+3*number+3] = FSR::P_FSR1_L;    // start address
        number++;
    }

    if(Ping(FSR::ID_R_FSR, 0) == SUCCESS)
    {
        m_BulkReadTxPacket[PARAMETER+3*number+1] = 10;               // length
        m_BulkReadTxPacket[PARAMETER+3*number+2] = FSR::ID_R_FSR;   // id
        m_BulkReadTxPacket[PARAMETER+3*number+3] = FSR::P_FSR1_L;    // start address
        number++;
    }

    m_BulkReadTxPacket[LENGTH]          = (number * 3) + 3;
}

int CM730::BulkRead()
{
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };

    if(m_BulkReadTxPacket[LENGTH] != 0)
        return TxRxPacket(m_BulkReadTxPacket, rxpacket, 0);
    else
    {
        MakeBulkReadPacket();
        return TX_FAIL;
    }
}

int CM730::SyncWrite(int start_addr, int each_length, int number, int *pParam)
{
	unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
	unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
	int n;

    txpacket[ID]                = (unsigned char)ID_BROADCAST;
    txpacket[INSTRUCTION]       = INST_SYNC_WRITE;
    txpacket[PARAMETER]			= (unsigned char)start_addr;
    txpacket[PARAMETER + 1]		= (unsigned char)(each_length - 1);
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

char* CM730::getTxRxErrorString(int error_code)
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

char* CM730::getInstructionTypeString(int instruction_value)
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