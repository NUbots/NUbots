/*! @file NUKick.h
    @brief Declaration of nukick class
 
    @class NUKick
    @brief A module to provide kicks
 
    @author Jed Rietveld
 
  Copyright (c) 2010 Jed Rietveld
 
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

#ifndef NUKICK_H
#define NUKICK_H

class NUSensorsData;
class NUActionatorsData;
class KickJob;
class NUWalk;
#include "Motion/NUMotionProvider.h"

#include "Kinematics/Kinematics.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include <string>

class FieldObjects;

class NUKick : public NUMotionProvider
{
public:
    enum KickingLeg
    {
        leftLeg,
        rightLeg,
        noLeg
    };

    static NUKick* getKick(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions);

    NUKick(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions);
    ~NUKick();

    void stop();
    void stopHead();
    void stopArms();
    void stopLegs();
    virtual void kill();

    virtual bool isActive();
    virtual bool isUsingHead();
    virtual bool isUsingArms();
    virtual bool isUsingLegs();
    bool isReady();

    virtual bool requiresHead() {return isUsingHead();}
    virtual bool requiresArms() {return isUsingArms();}
    virtual bool requiresLegs() {return true;}

    virtual void setArmEnabled(bool leftarm, bool rightarm);
    virtual void setHeadEnabled(bool head);

    //virtual void setKickParameters();
    virtual void loadKickParameters();
    void process(NUSensorsData* data, NUActionatorsData* actions);
    void process(KickJob* job);

    virtual void doKick() = 0;

    bool kickPossible(float ball_x, float ball_y, float target_x, float target_y);

    std::string toString(KickingLeg theLeg);


protected:
    virtual void kickToPoint(const vector<float>& position, const vector<float>& target);
    NUWalk* m_walk;                     //!< local pointer to the walk engine
    
    float m_ball_x;                    //!< the current ball x position relative to robot in cm
    float m_ball_y;                    //!< the current ball y position relative to robot in cm

    float m_target_x;
    float m_target_y;

    float m_defaultMotorGain;
    float m_defaultArmMotorGain;

    vector<float> m_initial_larm;
    vector<float> m_initial_rarm;
    vector<float> m_initial_lleg;
    vector<float> m_initial_rleg;

    KickingLeg m_kicking_leg;

    bool m_kick_ready;
    bool m_kick_enabled;                            //!< true if the kick is enabled, false otherwise
    bool m_head_enabled;                            //!< true if the kick is allowed to move the head
    bool m_larm_enabled;                            //!< true if the kick is allowed to move the left arm
    bool m_rarm_enabled;                            //!< true if the kick is allowed to move the right arm

    double m_currentTimestamp;
    double m_previousTimestamp;

    vector<Rectangle> m_kick_regions;

};


#endif

