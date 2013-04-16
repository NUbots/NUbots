#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <libgen.h>

#include <termios.h>
#include <term.h>
#include <pthread.h>

#include "LinuxDARwIn.h"
#include "FSR.h"
#include "mjpg_streamer.h"

using namespace Robot;

#define INI_FILE_PATH       "../../../../Data/config.ini"
#define U2D_DEV_NAME        "/dev/ttyUSB0"

void draw_target(Image* img, int x, int y, int r, int g, int b);

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

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

void* walk_thread(void* ptr)
{
    while(1) {
        int ch = _getch();
        if(ch == 0x20) {
            if(Walking::GetInstance()->IsRunning() == true) {
                MotionManager::GetInstance()->StopLogging();
                Walking::GetInstance()->Stop();
            }
            else {
                MotionManager::GetInstance()->StartLogging();
                Walking::GetInstance()->Start();
            }
        }
    }
    return NULL;
}

int main()
{
	printf( "\n===== FSR Tutorial for DARwIn =====\n\n");

    change_current_dir();
    minIni* ini = new minIni(INI_FILE_PATH);

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    Image* img_position = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
    Image* img_send = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    FILE *src;
    src = fopen("foot.raw", "rb");
    if(src != NULL)
    {
        fread(img_position->m_ImageData, 1, img_position->m_ImageSize, src);
        fclose(src);
    }

    //////////////////// Framework Initialize ////////////////////////////
    LinuxCM730 linux_cm730(U2D_DEV_NAME);
    CM730 cm730(&linux_cm730);
    if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        printf("Fail to initialize Motion Manager!\n");
            return 0;
    }
    Walking::GetInstance()->LoadINISettings(ini);
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();
    /////////////////////////////////////////////////////////////////////

    int n = 0;
    int param[JointData::NUMBER_OF_JOINTS * 5];
    int wGoalPosition, wStartPosition, wDistance;

    for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
    {
        wStartPosition = MotionStatus::m_CurrentJoints.GetValue(id);
        wGoalPosition = Walking::GetInstance()->m_Joint.GetValue(id);
        if( wStartPosition > wGoalPosition )
            wDistance = wStartPosition - wGoalPosition;
        else
            wDistance = wGoalPosition - wStartPosition;

        wDistance >>= 2;
        if( wDistance < 8 )
            wDistance = 8;

        param[n++] = id;
        param[n++] = CM730::GetLowByte(wGoalPosition);
        param[n++] = CM730::GetHighByte(wGoalPosition);
        param[n++] = CM730::GetLowByte(wDistance);
        param[n++] = CM730::GetHighByte(wDistance);
    }
    cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);

    printf("Press the ENTER key to begin!\n");
    getchar();
    printf("Press the SPACE key to start/stop walking.. \n\n");

    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
    MotionManager::GetInstance()->SetEnable(true);

    static const int MAX_FSR_VALUE = 254;
    int left_fsr_x, left_fsr_y, right_fsr_x, right_fsr_y;

    Walking::GetInstance()->LoadINISettings(ini);
    pthread_t thread_t;
    pthread_create(&thread_t, NULL, walk_thread, NULL);

    while(1)
    {
        printf("\r");

        /* Read & print FSR value */
//        printf(" L1:%5d", cm730.m_BulkReadData[FSR::ID_L_FSR].ReadWord(FSR::P_FSR1_L));
//        printf(" L2:%5d", cm730.m_BulkReadData[FSR::ID_L_FSR].ReadWord(FSR::P_FSR2_L));
//        printf(" L3:%5d", cm730.m_BulkReadData[FSR::ID_L_FSR].ReadWord(FSR::P_FSR3_L));
//        printf(" L4:%5d", cm730.m_BulkReadData[FSR::ID_L_FSR].ReadWord(FSR::P_FSR4_L));

        left_fsr_x = cm730.m_BulkReadData[FSR::ID_L_FSR].ReadByte(FSR::P_FSR_X);
        left_fsr_y = cm730.m_BulkReadData[FSR::ID_L_FSR].ReadByte(FSR::P_FSR_Y);
        printf(" LX:%3d", MAX_FSR_VALUE-left_fsr_x);
        printf(" LY:%3d", MAX_FSR_VALUE-left_fsr_y);

//        printf(" R1:%5d", cm730.m_BulkReadData[FSR::ID_R_FSR].ReadWord(FSR::P_FSR1_L));
//        printf(" R2:%5d", cm730.m_BulkReadData[FSR::ID_R_FSR].ReadWord(FSR::P_FSR2_L));
//        printf(" R3:%5d", cm730.m_BulkReadData[FSR::ID_R_FSR].ReadWord(FSR::P_FSR3_L));
//        printf(" R4:%5d", cm730.m_BulkReadData[FSR::ID_R_FSR].ReadWord(FSR::P_FSR4_L));

        right_fsr_x = cm730.m_BulkReadData[FSR::ID_R_FSR].ReadByte(FSR::P_FSR_X);
        right_fsr_y = cm730.m_BulkReadData[FSR::ID_R_FSR].ReadByte(FSR::P_FSR_Y);
        printf(" RX:%3d", right_fsr_x);
        printf(" RY:%3d", right_fsr_y);


        /* draw a position of ZMP */
        int r_position_x = (98*(MAX_FSR_VALUE-right_fsr_x)/MAX_FSR_VALUE) + 24;
        int r_position_y = (160*(MAX_FSR_VALUE-right_fsr_y)/MAX_FSR_VALUE) + 40;
        int l_position_x = (98*left_fsr_x/MAX_FSR_VALUE) + 198;
        int l_position_y = (160*left_fsr_y/MAX_FSR_VALUE) + 40;

        memcpy(img_send->m_ImageData, img_position->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
        if(left_fsr_x != 255 && left_fsr_y != 255)
            draw_target(img_send, l_position_x, l_position_y, 255, 0, 0);
        if(right_fsr_x != 255 && right_fsr_y != 255)
            draw_target(img_send, r_position_x, r_position_y, 255, 0, 0);

        if(left_fsr_x != 255 && left_fsr_y != 255 && right_fsr_x != 255 && right_fsr_y != 255)
            draw_target(img_send, (l_position_x+r_position_x)/2, (l_position_y+r_position_y)/2, 0, 0, 255);

        streamer->send_image(img_send);
    }

    return 0;
}

void draw_target(Image* img, int x, int y, int r, int g, int b)
{
    img->m_ImageData[(y-1)*img->m_WidthStep + x*img->m_PixelSize + 0] = r;
    img->m_ImageData[(y-1)*img->m_WidthStep + x*img->m_PixelSize + 1] = g;
    img->m_ImageData[(y-1)*img->m_WidthStep + x*img->m_PixelSize + 2] = b;

    img->m_ImageData[(y-2)*img->m_WidthStep + x*img->m_PixelSize + 0] = r;
    img->m_ImageData[(y-2)*img->m_WidthStep + x*img->m_PixelSize + 1] = g;
    img->m_ImageData[(y-2)*img->m_WidthStep + x*img->m_PixelSize + 2] = b;

    img->m_ImageData[(y-3)*img->m_WidthStep + x*img->m_PixelSize + 0] = r;
    img->m_ImageData[(y-3)*img->m_WidthStep + x*img->m_PixelSize + 1] = g;
    img->m_ImageData[(y-3)*img->m_WidthStep + x*img->m_PixelSize + 2] = b;

    img->m_ImageData[(y+1)*img->m_WidthStep + x*img->m_PixelSize + 0] = r;
    img->m_ImageData[(y+1)*img->m_WidthStep + x*img->m_PixelSize + 1] = g;
    img->m_ImageData[(y+1)*img->m_WidthStep + x*img->m_PixelSize + 2] = b;

    img->m_ImageData[(y+2)*img->m_WidthStep + x*img->m_PixelSize + 0] = r;
    img->m_ImageData[(y+2)*img->m_WidthStep + x*img->m_PixelSize + 1] = g;
    img->m_ImageData[(y+2)*img->m_WidthStep + x*img->m_PixelSize + 2] = b;

    img->m_ImageData[(y+3)*img->m_WidthStep + x*img->m_PixelSize + 0] = r;
    img->m_ImageData[(y+3)*img->m_WidthStep + x*img->m_PixelSize + 1] = g;
    img->m_ImageData[(y+3)*img->m_WidthStep + x*img->m_PixelSize + 2] = b;


    img->m_ImageData[y*img->m_WidthStep + (x-1)*img->m_PixelSize + 0] = r;
    img->m_ImageData[y*img->m_WidthStep + (x-1)*img->m_PixelSize + 1] = g;
    img->m_ImageData[y*img->m_WidthStep + (x-1)*img->m_PixelSize + 2] = b;

    img->m_ImageData[y*img->m_WidthStep + (x-2)*img->m_PixelSize + 0] = r;
    img->m_ImageData[y*img->m_WidthStep + (x-2)*img->m_PixelSize + 1] = g;
    img->m_ImageData[y*img->m_WidthStep + (x-2)*img->m_PixelSize + 2] = b;

    img->m_ImageData[y*img->m_WidthStep + (x-3)*img->m_PixelSize + 0] = r;
    img->m_ImageData[y*img->m_WidthStep + (x-3)*img->m_PixelSize + 1] = g;
    img->m_ImageData[y*img->m_WidthStep + (x-3)*img->m_PixelSize + 2] = b;

    img->m_ImageData[y*img->m_WidthStep + (x+1)*img->m_PixelSize + 0] = r;
    img->m_ImageData[y*img->m_WidthStep + (x+1)*img->m_PixelSize + 1] = g;
    img->m_ImageData[y*img->m_WidthStep + (x+1)*img->m_PixelSize + 2] = b;

    img->m_ImageData[y*img->m_WidthStep + (x+2)*img->m_PixelSize + 0] = r;
    img->m_ImageData[y*img->m_WidthStep + (x+2)*img->m_PixelSize + 1] = g;
    img->m_ImageData[y*img->m_WidthStep + (x+2)*img->m_PixelSize + 2] = b;

    img->m_ImageData[y*img->m_WidthStep + (x+3)*img->m_PixelSize + 0] = r;
    img->m_ImageData[y*img->m_WidthStep + (x+3)*img->m_PixelSize + 1] = g;
    img->m_ImageData[y*img->m_WidthStep + (x+3)*img->m_PixelSize + 2] = b;
}

