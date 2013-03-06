#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <term.h>
#include <ncurses.h>
#include <libgen.h>
#include "hex2bin.h"

#include <getopt.h>
#include <iostream>

#include "LinuxDARwIn.h"

using namespace Robot;


int _getch()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr( STDIN_FILENO, &oldt );
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
    return ch;
}

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

struct termios oldterm, new_term;
void set_stdin(void)
{
    tcgetattr(0,&oldterm);
    new_term = oldterm;
    new_term.c_lflag &= ~(ICANON | ECHO | ISIG); // 의미는 struct termios를 찾으면 됨.
    new_term.c_cc[VMIN] = 1;
    new_term.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_term);
}

void reset_stdin(void)
{
    tcsetattr(0, TCSANOW, &oldterm);
}

void change_current_dir()
{
    int r = 0;
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        r = chdir(dirname(exepath));
}

void help(char *progname)
{
    fprintf(stderr, "-----------------------------------------------------------------------\n");
    fprintf(stderr, "Usage: %s\n" \
                    " [-h | --help]........: display this help\n" \
                    " [-d | --device]......: port to open                     (/dev/ttyUSB0)\n" \
                    " [-c | --controller]..: controller firmware file       (cm730_0x13.hex)\n" \
                    " [-a | --actuator]....: actuator firmware file (mx28_0x1E+FSR_0x11.hex)\n", progname);
    fprintf(stderr, "-----------------------------------------------------------------------\n");
    fprintf(stderr, "Example #1:\n" \
                    " To open a default port and install with firmware file \"cm730.hex\":\n" \
                    "  %s -c cm730.hex\n", progname);
    fprintf(stderr, "-----------------------------------------------------------------------\n");
    fprintf(stderr, "Example #2:\n" \
                    " To open a /dev/ttyUSB1 and install with default firmware file name:\n" \
                    "  %s -d /dev/ttyUSB1 \n", progname);
    fprintf(stderr, "-----------------------------------------------------------------------\n");
}

void Reset(CM730 *cm730, int id)
{
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

int main(int argc, char *argv[])
{
    int r = 0;

    fprintf(stderr, "\n***********************************************************************\n");
    fprintf(stderr,   "*             CM-730 & Actuator & FSR Firmware Installer              *\n");
    fprintf(stderr,   "***********************************************************************\n\n");

    char *controller_fw = (char*)"cm730_0x13.hex";
    char *actuator_fw = (char*)"mx28_0x1E+FSR_0x11.hex";
    char *dev = (char*)"/dev/ttyUSB0";

    /* parameter parsing */
    while(1)
    {
        int option_index = 0, c = 0;
        static struct option long_options[] = {
                {"h", no_argument, 0, 0},
                {"help", no_argument, 0, 0},
                {"d", required_argument, 0, 0},
                {"device", required_argument, 0, 0},
                {"c", required_argument, 0, 0},
                {"controller", required_argument, 0, 0},
                {"a", required_argument, 0, 0},
                {"actuator", required_argument, 0, 0},
                {0, 0, 0, 0}
        };

        /* parsing all parameters according to the list above is sufficent */
        c = getopt_long_only(argc, argv, "", long_options, &option_index);

        /* no more options to parse */
        if(c == -1) break;

        /* unrecognized option */
        if(c == '?') {
            help(argv[0]);
            return 0;
        }

        /* dispatch the given options */
        switch(option_index) {
        /* h, help */
        case 0:
        case 1:
            help(argv[0]);
            return 0;
            break;

            /* d, device */
        case 2:
        case 3:
            dev = strdup(optarg);
            break;

            /* c, controller */
        case 4:
        case 5:
            controller_fw = strdup(optarg);
            break;

            /* a, actuator */
        case 6:
        case 7:
            actuator_fw = strdup(optarg);
            break;

        default:
            help(argv[0]);
            return 0;
        }
    }

    fprintf(stderr, "You can choose to: \n\n"
                    "  (1) CM-730 firmware installation.    (with \"%s\")\n" \
                    "  (2) Dynamixel firmware installation. (with \"%s\")\n\n", controller_fw, actuator_fw);
    char choice = 0;
    do{
        fprintf(stderr, "Enter your choice: ");
        std::cin >> choice;
    }while(choice != '1' && choice != '2');
    int mode = atoi(&choice);

    unsigned char binMem[MEMORY_MAXSIZE] = { 0xff, };
    long startAddr;
    long binSize = 0;

    if(mode == 1)
    {
        fprintf(stderr, "\n [ CM-730 firmware installation mode ]\n\n");
        fprintf(stderr, "Load %s... ", controller_fw);

        if(hex2bin(controller_fw, binMem, &startAddr, &binSize) == false)
            return 0;
        fprintf(stderr, "Success!! \nBinary size: %ld byte\n\n", binSize);
    }
    else if(mode == 2)
    {
        fprintf(stderr, "\n [ Dynamixel firmware installation mode ]\n\n");
        fprintf(stderr, "Load %s... ", actuator_fw);

        if(hex2bin(actuator_fw, binMem, &startAddr, &binSize) == false)
            return 0;
        fprintf(stderr, "Success!! \nBinary size: %ld byte\n\n", binSize);
    }

    // Open port
    int fd;
    struct termios oldtio, newtio;
    char TxCh;
    char RcvBuff[256] = { 0, };
    int RcvNum;

    if((fd = open(dev, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0)
    {
        fprintf(stderr, "Fail to open port %s\n", dev);
        return 0;
    }

    fprintf(stderr, "[ESC] : Quit program \n");

    tcgetattr(fd, &oldtio);
    memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag      = B57600|CS8|CLOCAL|CREAD;
    newtio.c_iflag      = IGNPAR;
    newtio.c_oflag      = 0;
    newtio.c_lflag      = 0;
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN]   = 0;
    tcsetattr(fd, TCSANOW, &newtio);
    tcflush(fd, TCIFLUSH);

    set_stdin();

    fprintf(stderr, "Press DARwIn-OP's Reset button to start...\n");

    while(1)
    {
        r = write(fd, "#", 1);
        usleep(20000);
        RcvNum = read(fd, RcvBuff, 256);
        if(RcvNum > 0)
        {
            RcvBuff[RcvNum] = 0;
            if(strcmp(RcvBuff, "#") == 0)
            {
                r = write(fd, "\r", 1);
                break;
            }
        }

        if(kbhit())
        {
            TxCh = _getch();
            if(TxCh == 0x1b)
                goto EXIT;
        }
    }

    if(mode == 1)
    {
        /*+++ start download +++*/
        r = write(fd, "l\r", 2);

        while(1)
        {
            usleep(135000);
            RcvNum = read(fd, RcvBuff, 256);
            if(RcvNum > 0)
            {
                RcvBuff[RcvNum] = 0;
                fprintf(stderr, "%s", RcvBuff);
            }
            else
                break;
        }

        fprintf(stderr, "\nErase block complete...\n");
        usleep(100000);


        unsigned char bytesum = 0x00;
        for(long n=0; n<128*1024; n++)
            bytesum += binMem[startAddr + n];

        printf("\n");
        long size = 0;
        long max_unit = 64;
        long unit;
        int res;
        while(size < binSize)
        {
            unit = binSize - size;
            if(unit > max_unit)
                unit = max_unit;

            res = write(fd, &binMem[startAddr + size], unit);
            if(res > 0)
            {
                size += res;
                printf("\rDownloading Firmware (%ld/%ld byte)", size, binSize);
            }
        }
        do
        {
            res = write(fd, &bytesum, 1);
        }while(res < 0);

        printf("\nDownloading Bytesum:%2X\n", bytesum);

        for(int x = 0; x < 100; x++)
        {
            usleep(10000);
            RcvNum = read(fd, RcvBuff, 256);
            if(RcvNum > 0)
            {
                RcvBuff[RcvNum] = 0;
                fprintf(stderr, "%s", RcvBuff);
            }
        }
        /*--- end download ---*/

        usleep(10000);

        r = write(fd, "\rgo\r", 4); // Exit bootloader
        usleep(50000);
        RcvNum = read(fd, RcvBuff, 256);
        if(RcvNum > 0)
        {
            RcvBuff[RcvNum] = 0;
            printf("%s", RcvBuff);
        }
    }
    else if(mode == 2)
    {
        /*+++ start download +++*/
        r = write(fd, "l 8023000\r", 10);

        while(1)
        {
            usleep(135000);
            RcvNum = read(fd, RcvBuff, 256);
            if(RcvNum > 0)
            {
                RcvBuff[RcvNum] = 0;
                fprintf(stderr, "%s", RcvBuff);
            }
            else
                break;
        }

        fprintf(stderr, "\nErase block complete...\n");
        usleep(100000);

        unsigned char bytesum = 0x00;
        for(long n=0; n<binSize; n++)
            bytesum += binMem[startAddr + n];

        printf("\n");
        long size = 0;
        long max_unit = 64;
        long unit;
        int res;
        while(size < binSize)
        {
            unit = binSize - size;
            if(unit > max_unit)
                unit = max_unit;

            res = write(fd, &binMem[startAddr + size], unit);
            if(res > 0)
            {
                size += res;
                printf("\rDownloading Firmware (%ld/%ld byte)", size, binSize);
            }
        }
        do
        {
            res = write(fd, &bytesum, 1);
        }while(res < 0);

        printf("\nDownloading Bytesum:%2X\n", bytesum);

        for(int x = 0; x < 100; x++)
        {
            usleep(10000);
            RcvNum = read(fd, RcvBuff, 256);
            if(RcvNum > 0)
            {
                RcvBuff[RcvNum] = 0;
                fprintf(stderr, "%s", RcvBuff);
            }
        }
        /*--- end download ---*/

        r = write(fd, "go 8023000", 10);
        r = write(fd, "\r", 1);

        int wait_count = 0;
        char last_char = 0;
        while(1)
        {
            if(kbhit())
            {
                TxCh = _getch();
                if(TxCh == 0x1b)
                    break;
                else if(TxCh == 127) // backspace
                    TxCh = 0x08; // replace backspace value

                r = write(fd, &TxCh, 1);
            }

            RcvNum = read(fd, RcvBuff, 256);
            if(RcvNum > 0)
            {
                RcvBuff[RcvNum] = 0;
                printf("%s", RcvBuff);
                last_char = RcvBuff[RcvNum-1];
                wait_count = 0;
            }
            else
                wait_count++;

            if(wait_count > 200 && last_char == '!')
                break;

            usleep(20000);
        }

        reset_stdin();

        tcsetattr(fd, TCSANOW, &oldtio);
        close(fd);

        fprintf(stderr, "\n\n");
        LinuxCM730 linux_cm730(dev);
        CM730 cm730(&linux_cm730);
        if(cm730.Connect() == true)
        {
            int firm_ver = 0;
            if(cm730.ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, 0)  != CM730::SUCCESS)
            {
                fprintf(stderr, "Can't read firmware version from Dynamixel ID %d!! \n\n", JointData::ID_HEAD_PAN);
                goto EXIT;
            }

#ifdef MX28_1024
            if(27 <= firm_ver)
            {
                fprintf(stderr, "\n MX-28's firmware is not support 1024 resolution!! \n");
                fprintf(stderr, " Remove '#define MX28_1024' from 'MX28.h' file and rebuild.\n\n");
                goto EXIT;
            }
#else
            if(0 < firm_ver && firm_ver < 27)
            {
                fprintf(stderr, "\n MX-28's firmware is not support 4096 resolution!! \n");
                fprintf(stderr, " Upgrade MX-28's firmware to version 27(0x1B) or higher.\n\n");
                goto EXIT;
            }
#endif
            for(int i=JointData::ID_R_SHOULDER_PITCH; i<JointData::NUMBER_OF_JOINTS; i++)
                Reset(&cm730, i);

            Reset(&cm730, CM730::ID_CM);
        }
        else
            fprintf(stderr, "CM-730 Connect fail!! \n");

        return 0;
    }

    EXIT:
    reset_stdin();

    tcsetattr(fd, TCSANOW, &oldtio);
    printf("\nTerminated program\n\n");
    return 0;
}
