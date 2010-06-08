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

#include "./Kicks/IK.h"
#include <stack>
#include "Kinematics/Kinematics.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"

enum poseType {DO_NOTHING, USE_LEFT_LEG, USE_RIGHT_LEG, LIFT_LEG, ADJUST_YAW, SET_LEG, POISE_LEG, SWING, RETRACT, REALIGN_LEGS, UNSHIFT_LEG, RESET, NO_KICK, PRE_KICK, POST_KICK, TRANSFER_TO_SUPPORT};

class NUKick
{
    enum legId_t
    {
        leftLeg,
        rightLeg,
        noLeg
    };

public:
    NUKick(NUWalk* walk);
    ~NUKick();
    void stop();
    void kill();
    void loadKickParameters();
    void process(NUSensorsData* data, NUActionatorsData* actions);
    void process(KickJob* job);
    bool isActive();
private:
    vector<float> bestKickingPosition(const vector<float>& ballPosition,const vector<float>& targetPositon);
    void kickToPoint(const vector<float>& position, const vector<float>& target);
    void preKick();
    void doKick();
    bool doPreKick();
    bool doPostKick();
    bool chooseLeg();
    bool liftLeg();
    bool adjustYaw();
    bool positionFoot();
    bool setLeg();
    bool poiseLeg();
    bool swing();
    bool retract();
    void postKick();
    bool ShiftWeightToFoot(legId_t supportLeg, float targetWeightPercentage, float speed);
    bool LiftKickingLeg(legId_t kickingLeg);
    bool SwingLegForward(legId_t kickingLeg, float speed);
    bool LowerLeg(legId_t kickingLeg);
    void BalanceCoP(vector<float>& jointAngles, float CoPx, float CoPy);
    void FlattenFoot(vector<float>& jointAngles);
    bool IsPastTime(float time);
    void MaintainSwingHeight(legId_t supportLeg, vector<float>& supportLegJoints, legId_t swingLeg, vector<float>& swingLegJoints, float swingHeight);
    double TimeBetweenFrames();
    float perSec2perFrame(float value);
    float SpeedMultiplier();
    float GainMultiplier();
    void MoveLimbToPositionWithSpeed(NUActionatorsData::bodypart_id_t limbId, vector<float> currentPosition, vector<float> targetPosition, float maxSpeed , float gain);

//private:
    NUSensorsData* m_data;              //!< local pointer to the latest sensor data
    NUActionatorsData* m_actions;       //!< local pointer to the next actionators data
    NUWalk* m_walk;                     //!< local pointer to the walk engine
    Kinematics* kinematicModel;
    
    float m_ball_x;                    //!< the current ball x position relative to robot in cm
    float m_ball_y;                    //!< the current ball y position relative to robot in cm

    float m_target_x;
    float m_target_y;
    double m_target_timestamp;

    vector<double> poseData;
    poseType pose;
    bool lock;

    float m_defaultMotorGain;
    vector<float> m_leftLegInitialPose;
    vector<float> m_rightLegInitialPose;

    stack<vector<double> > poseStack;
    legId_t m_kickingLeg;
    Legs * IKSys;
    bool m_kickIsActive;

    bool m_stateCommandGiven;
    double m_currentTimestamp;
    double m_previousTimestamp;
};

#endif

