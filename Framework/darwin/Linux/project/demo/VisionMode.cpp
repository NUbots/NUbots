/*
 * VisionMode.cpp
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom
 */
#include <stdio.h>
#include "VisionMode.h"
#include "Action.h"
#include "ColorFinder.h"
#include "LinuxActionScript.h"

namespace Robot
{

void VisionMode::Play(int color)
{
    static int old_color = 0, color_count = 0;

    if(old_color != color || color == 0)
    {
        old_color = color;
        color_count = 0;
    }
    else
        color_count++;

    if(color_count < 15) return;

    switch(color)
    {
    case (RED):
        Action::GetInstance()->Start(4);
        LinuxActionScript::PlayMP3("../../../Data/mp3/Thank you.mp3");
        break;
    case (YELLOW):
        Action::GetInstance()->Start(41);
        LinuxActionScript::PlayMP3("../../../Data/mp3/Introduction.mp3");
        break;
    case (BLUE):
        Action::GetInstance()->Start(24);
        LinuxActionScript::PlayMP3("../../../Data/mp3/Wow.mp3");
        break;
    case (RED|YELLOW):
        Action::GetInstance()->Start(38);
        LinuxActionScript::PlayMP3("../../../Data/mp3/Bye bye.mp3");
        break;
    case (RED|BLUE):
        Action::GetInstance()->Start(54);
        LinuxActionScript::PlayMP3("../../../Data/mp3/Clap please.mp3");
        break;
    case (BLUE|YELLOW):
        Action::GetInstance()->Start(15);
        LinuxActionScript::PlayMP3("../../../Data/mp3/Sit down.mp3");
        while(Action::GetInstance()->IsRunning()) usleep(8*1000);

        Action::GetInstance()->Start(1);
        LinuxActionScript::PlayMP3("../../../Data/mp3/Stand up.mp3");
        break;
    case (RED|YELLOW|BLUE):
        Action::GetInstance()->Start(27);
        LinuxActionScript::PlayMP3("../../../Data/mp3/Oops.mp3");
        break;
    }

    color_count = 0;
}

}
