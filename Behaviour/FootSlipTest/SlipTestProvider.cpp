/*! @file SlipTestProvider.cpp
    @brief Implementation of slip testing behaviour class

    @author Josiah Walker
 
 Copyright (c) 2012 Josiah Walker
 
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

#include "SlipTestProvider.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/GameInformation/GameInformation.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/MotionFreezeJob.h"

#include "Infrastructure/FieldObjects/FieldObjects.h"

#include "Behaviour/BehaviourPotentials.h"

#include <math.h>
#include <numeric>
#include "debug.h"
#include "debugverbositybehaviour.h"
#include "Tools/Math/StlVector.h"
#include "Tools/Math/Vector2.h"
#include "Tools/Math/General.h"
#include "Tools/Math/Sampling.h"

using namespace std;

SlipTestProvider::SlipTestProvider(Behaviour* manager) : BehaviourProvider(manager)
{
    cout << "Sample ID, Start X, Start Y, Start Heading, End X, End Y, End Heading, Trans Speed, Trans Direction, Turn Speed, X-odometry, Y-odometry, Bearing-odometry, Time Taken" << endl << flush;
    samplePointIndex = 0;
    vector<float> walk = Sampling::HaltonPointND(samplePointIndex,3);
    m_trans_speed = walk[0]*2.-1.;
    m_trans_direction = walk[1]*6.-3.;
    m_turn_speed = walk[2]*2.-1.;
    
    m_start_x = 0.;
    m_start_y = 0.;
    m_start_heading = 0.;
    m_return_to_start = true;
    m_run_start = 0.;
    m_odomX = 0.;
    m_odomY = 0.;
    m_odomB = 0.;
    //Self& self = m_field_objects->self;
    //hist_gpsX.push_back(self.wmX());
    //hist_gpsY.push_back(self.wmY());
    //hist_gpsB.push_back(self.Heading());
}


SlipTestProvider::~SlipTestProvider()
{
}

void SlipTestProvider::doBehaviour()
{
    // hack it, and put the GameState into Playing
    while (m_game_info->getCurrentState() != GameInformation::PlayingState)
        m_game_info->doManualStateChange();
    
    Self& self = m_field_objects->self;
    
    //get this segment of odometry
    vector<float> odometry;
    Blackboard->Sensors->getOdometry(odometry);
    
    //save odometry history
    hist_odomX.push_back(odometry[0]);
    hist_odomY.push_back(odometry[1]);
    hist_odomB.push_back(odometry[2]);
    hist_gpsX.push_back(self.wmX());
    hist_gpsY.push_back(self.wmY());
    hist_gpsB.push_back(self.Heading());
    cout << odometry[0] << ", " << odometry[1] << ", " << odometry[2] << ", " << self.wmX() << ", " << self.wmY() << ", " << self.Heading() << endl << flush;
     
    //return to the start location
    if (m_return_to_start) {
        float dpx = self.wmX() - 150.;
        float dpy = self.wmY();
        vector<float> fieldstate;
        fieldstate.push_back(150.);
        fieldstate.push_back(0.);
        fieldstate.push_back(0.);
        vector<float> walkdirection = BehaviourPotentials::goToFieldState(self,fieldstate);
        Blackboard->Jobs->addMotionJob(new WalkJob(walkdirection[0]/1.2,walkdirection[1],walkdirection[2]/2.));
        //cout << "going to start: " << dpx*dpx+dpy*dpy << endl << flush;
        
        //change values
        
        if (dpx*dpx+dpy*dpy < 225.) {
            m_return_to_start = false;
            m_run_start = Blackboard->Sensors->CurrentTime;
            samplePointIndex++;
            vector<float> walk = Sampling::HaltonPointND(samplePointIndex,3);
            m_trans_speed = walk[0]*2.-1.;
            m_trans_direction = walk[1]*6.-3.;
            m_turn_speed = walk[2]*2.-1.;
        }
    
    //do a test run
    } else {
        
        //stand still for a bit at the start
        if (m_run_start < Blackboard->Sensors->CurrentTime-2000. && m_run_start > Blackboard->Sensors->CurrentTime-6500.) {
            Blackboard->Jobs->addMotionJob(new WalkJob(m_trans_speed,m_trans_direction,m_turn_speed));
            //cout << "walking around";
            
            
            
            //sum odometry
            m_odomX += odometry[0]*cos(m_odomB) + odometry[1]*sin(m_odomB);
            m_odomY += odometry[1]*cos(m_odomB) + odometry[0]*sin(m_odomB);
            m_odomB += odometry[2];
            
        } else if (m_run_start < Blackboard->Sensors->CurrentTime-6500. && m_run_start > Blackboard->Sensors->CurrentTime-7000.) {
            Blackboard->Jobs->addMotionJob(new MotionFreezeJob());
        } else {
            Blackboard->Jobs->addMotionJob(new WalkJob(0.,0.,0.));
            m_start_x = self.wmX();
            m_start_y = self.wmY();
            m_start_heading = self.Heading();
        }
        
        //we are finished this run, get data
        if (m_run_start < Blackboard->Sensors->CurrentTime-7000.) {
            
            //cout << samplePointIndex << ", " << m_start_x << ", " << m_start_y << ", " << m_start_heading << ", " << self.wmX() << ", " << self.wmY() << ", " << self.Heading() << ", " << m_trans_speed << ", " << m_trans_direction << ", " << m_turn_speed << ", " << m_odomX << ", " << m_odomY << ", " << m_odomB << ", " << Blackboard->Sensors->CurrentTime-m_run_start-2000. << endl << flush;
            m_return_to_start = true;
            m_odomX = 0.;
            m_odomY = 0.;
            m_odomB = 0.;
            
            if (samplePointIndex % 20 == 0) {
            vector<float> odom_gpsX,odom_gpsY,odom_gpsB;
            odom_gpsX.push_back(0.);
            odom_gpsY.push_back(0.);
            odom_gpsB.push_back(0.);
            for (int i = 1; i < hist_gpsX.size(); i++) {
                float dx = hist_gpsX[i]-hist_gpsX[i-1];
                float dy = hist_gpsY[i]-hist_gpsY[i-1];
                float db = hist_gpsB[i]-hist_gpsB[i-1];
                odom_gpsX.push_back(dx*cos(hist_gpsB[i])+dy*sin(hist_gpsB[i]));
                odom_gpsY.push_back(dy*cos(hist_gpsB[i])+dx*sin(hist_gpsB[i]));
                odom_gpsB.push_back(db);
                
                cout << hist_odomX[i] << ", " << hist_odomY[i] << ", " << hist_odomB[i] << ", " << odom_gpsX[i] << ", " << odom_gpsY[i] << ", " << odom_gpsB[i] << endl << flush;
            }
            
            cout << "<X>, <Y>, <B>" << endl;
            //predict realX
            cout << Sampling::Covariance1D(hist_odomX,odom_gpsX)/Sampling::Variance1D(hist_odomX);
            cout << ", ";
            cout << Sampling::Covariance1D(hist_odomY,odom_gpsX)/Sampling::Variance1D(hist_odomY);
            cout << ", ";
            cout << Sampling::Covariance1D(hist_odomB,odom_gpsX)/Sampling::Variance1D(hist_odomB);
            cout << ", " << endl;
            
            //[redict realY
            cout << Sampling::Covariance1D(hist_odomX,odom_gpsY)/Sampling::Variance1D(hist_odomX);
            cout << ", ";
            cout << Sampling::Covariance1D(hist_odomY,odom_gpsY)/Sampling::Variance1D(hist_odomY);
            cout << ", ";
            cout << Sampling::Covariance1D(hist_odomB,odom_gpsY)/Sampling::Variance1D(hist_odomB);
            cout << ", " << endl;
            
            //predict realB
            cout << Sampling::Covariance1D(hist_odomX,odom_gpsB)/Sampling::Variance1D(hist_odomX);
            cout << ", ";
            cout << Sampling::Covariance1D(hist_odomY,odom_gpsB)/Sampling::Variance1D(hist_odomY);
            cout << ", ";
            cout << Sampling::Covariance1D(hist_odomB,odom_gpsB)/Sampling::Variance1D(hist_odomB);
            cout << ", " << endl << flush;
            
            }
            
            
        }
    }
}



