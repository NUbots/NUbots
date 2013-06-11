/*! @file AnalsyseEnvironmentState.h
    @brief 

    @author Jason Kulk, Aaron Wong
 
  Copyright (c) 2011 Jason Kulk, Aaron Wong
 
    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ANALYSEENVIRONMENTSTATE_H
#define ANALYSEENVIRONMENTSTATE_H

#include "Behaviour/BehaviourState.h"
#include "EnvironmentalEmotionsProvider.h"
#include "EnvironmentalEmotionsState.h"


#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"

#include "Infrastructure/NUImage/ColorModelConversions.h"
#include "Infrastructure/NUImage/NUImage.h"

#include "debug.h"
#include <math.h>

class AnalyseEnvironmentState : public EnvironmentalEmotionsState
{
public:
    double timelastplayedsound;
    AnalyseEnvironmentState(EnvironmentalEmotionsProvider * parent)  : EnvironmentalEmotionsState(parent)  
    {
        timelastplayedsound=0;
        //debug << "Init Analsyse State: ";
    };
    virtual ~AnalyseEnvironmentState() {};
        
    virtual BehaviourState* nextState(){return this;};
        
    virtual void doState()
    {
        
        //m_jobs->addMotionJob(new WalkJob(0,0,-0.3));
        m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Localisation));
        //Wait 30seconds: then say something:
        if(m_data->GetTimestamp() - timelastplayedsound > 15000) //30,000 = 30Seconds
        {
            debug << "Analsyse Environment [DO]: ";
            //process IMAGE to get averageColour:
            std::string word = ProcessImage();
            //std::string word = "relaxed";


            //int index = (rand())%(numEmotions-1);
            //debug << "Array Index: "<<index;
            //debug << word;
            std::vector<std::string> sounds (2);
            sounds[0] = "EmotionalWords/IFeel.wav";
            sounds[1] = "EmotionalWords/" + word + ".wav";
            m_actions->add(NUActionatorsData::Sound, m_data->GetTimestamp(), sounds);
            timelastplayedsound = m_data->GetTimestamp();
        }
        return;
        //m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::BallAndLocalisation, 40, 90000, -0.8, 0.8));
    };
    virtual std::string ProcessImage()
    {
        unsigned char R,G,B;
        double Tr = 0;
        double Tg = 0;
        double Tb = 0;
        int numRandomPoints = 10000;
        int i = 0;
        std::string word;
        debug << "Colours: "<< Tr << Tg << Tb;
        while (i < numRandomPoints)
        {
            Pixel pixel = (*Blackboard->Image)(rand()%(Blackboard->Image->getWidth()-1),rand()%(Blackboard->Image->getHeight()-1));
            ColorModelConversions::fromYCbCrToRGB(pixel.y, pixel.cb,pixel.cr,R,G,B);
            Tr = Tr + R;
            Tg = Tg + G;
            Tb = Tb + B;
            i++;
        }
        debug << "Colours: "<< Tr << Tg << Tb;
        //RED WINS: "HIGH"
        if(Tr >= Tb && Tr >= Tg)
        {
            debug << "RED: "<< Tr << Tg << Tb;
            int numEmotions = 5;
            std::string emotions[] = {"cheerful","anxious","excited","surprised","angry"};
            int index = rand()%(numEmotions);
            word = emotions[index];
        }
        //BLUE WINS: "LOW"
        else if (Tb >= Tr && Tb >=Tg) 
        {
            debug << "BLUE: "<< Tr << Tg << Tb;
            int numEmotions = 4;
            std::string emotions[] = {"scared","tense","nervous","horrified"};
            int index = rand()%(numEmotions);
            word = emotions[index];
        }
        //GREEN WINS: "MEDIUM"
        else
        {
            debug << "GREEN: "<< Tr << Tg << Tb;
            int numEmotions = 2;
            std::string emotions[] = {"happy","relaxed"};
            int index = rand()%(numEmotions);
            word =  emotions[index];
        }
        return word;
        //debug << "NOTHING HAPPENED";
        //return "relaxed";
    };
};



#endif

