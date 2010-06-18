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
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include <string>

class NUKick : public NUMotionProvider
{
    enum poseType_t
    {
        DO_NOTHING,
        LIFT_LEG,
        ADJUST_YAW,
        SET_LEG,
        POISE_LEG,
        SWING,
        RETRACT,
        REALIGN_LEGS,
        UNSHIFT_LEG,
        ALIGN_BALL,
        ALIGN_SIDE,
        EXTEND_SIDE,
        RESET,
        NO_KICK,
        PRE_KICK,
        POST_KICK,
        TRANSFER_TO_SUPPORT,
        numPoses
    };

    enum swingDirection_t
    {
        ForwardSwing,
        LeftSwing,
        RightSwing,
        numStyles
    };

    enum legId_t
    {
        leftLeg,
        rightLeg,
        noLeg
    };

public:
    NUKick(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions);
    ~NUKick();
    void stop();
    void stopHead();
    void stopArms();
    void stopLegs();
    void kill();
    
    bool isActive();
    bool isUsingHead();
    bool isUsingArms();
    bool isUsingLegs();
    
    bool requiresHead() {return true;}
    bool requiresArms() {return true;}
    bool requiresLegs() {return true;}
    
    void loadKickParameters();
    void process(NUSensorsData* data, NUActionatorsData* actions);
    void process(KickJob* job);
    std::string toString(legId_t theLeg);
    std::string toString(swingDirection_t theSwingDirection);
    std::string toString(poseType_t thePose);
private:
    vector<float> bestKickingPosition(const vector<float>& ballPosition,const vector<float>& targetPositon);
    void kickToPoint(const vector<float>& position, const vector<float>& target);
    void preKick();
    void doKick();
    bool doPreKick();
    bool doPostKick();
    bool doPoise(legId_t leg, float angleChange, float speed);

    bool chooseLeg();
    bool kickAbortCondition();
    bool ShiftWeightToFoot(legId_t supportLeg, float targetWeightPercentage, float speed, float time);
    bool ShiftWeightToFootClosedLoop(legId_t supportLeg, float targetWeightPercentage, float speed);
    bool LiftKickingLeg(legId_t kickingLeg, float speed);
    bool SwingLegForward(legId_t kickingLeg, float speed);
    bool SwingLegSideward(legId_t kickingLeg, float speed);
    bool AlignYposition(legId_t kickingLeg, float speed, float yPos);
    bool AlignXposition(legId_t kickingLeg, float speed, float xPos);
    bool LowerLeg(legId_t kickingLeg, float speed);
    bool BalanceCoP(legId_t supportLeg, float targetX = 0.0f, float targetY = 0.0f);
    void BalanceCoPLevelTorso(vector<float>& jointAngles, float CoPx, float CoPy, float targetX = 0.0f, float targetY = 0.0f);
    void BalanceCoPHipAndAnkle(vector<float>& jointAngles, float CoPx, float CoPy, float targetX = 0.0f, float targetY = 0.0f);
    void BalanceCoPHip(vector<float>& jointAngles, float CoPx, float CoPy = 0.0f);
    void BalanceCoPAnkle(vector<float>& jointAngles, float CoPx, float CoPy = 0.0f);
    void FlattenFoot(vector<float>& jointAngles);
    float FlatFootAnklePitch(float hipPitch, float kneePitch);
    float FlatFootAnkleRoll(float hipRoll);
    bool IsPastTime(float time);
    void MaintainSwingHeight(legId_t supportLeg, vector<float>& supportLegJoints, legId_t swingLeg, vector<float>& swingLegJoints, float swingHeight);
    double TimeBetweenFrames();
    float perSec2perFrame(float value);
    float SpeedMultiplier();
    float GainMultiplier();
    double MoveLimbToPositionWithSpeed(NUActionatorsData::bodypart_id_t limbId, vector<float> currentPosition, vector<float> targetPosition, float maxSpeed , float gain, float smoothness = 0.5);

    float CalculateForwardSwingSpeed(float kickDistance);
    float CalculateSidewardSwingSpeed(float kickDistance);

    void MoveArmsToKickPose(legId_t leadingArmleg, float speed);

//private:
    NUWalk* m_walk;                     //!< local pointer to the walk engine
    Kinematics* m_kinematicModel;
    
    float m_ball_x;                    //!< the current ball x position relative to robot in cm
    float m_ball_y;                    //!< the current ball y position relative to robot in cm

    float m_target_x;
    float m_target_y;
    double m_target_timestamp;

    poseType_t pose;
    bool lock;

    float m_defaultMotorGain;
    vector<float> m_leftLegInitialPose;
    vector<float> m_rightLegInitialPose;

//    stack<vector<double> > poseStack;
    legId_t m_kickingLeg;
    swingDirection_t m_swingDirection;
    //Legs * IKSys;
    bool m_kickIsActive;

    bool m_stateCommandGiven;
    double m_estimatedStateCompleteTime;
    double m_currentTimestamp;
    double m_previousTimestamp;

    class jointLimit
    {
    public:
        jointLimit(): min(0.0f), max(0.0f){};
        jointLimit(float minAngle, float maxAngle): min(minAngle), max(maxAngle){};
        float min;
        float max;
    };

    bool LimitJoints(legId_t leg, vector<float> jointPositions);
    vector<jointLimit> m_leftLegLimits;
    vector<jointLimit> m_rightLegLimits;
    Rectangle LeftFootForwardKickableArea;
    Rectangle RightFootForwardKickableArea;
    Rectangle LeftFootLeftKickableArea;
    Rectangle RightFootLeftKickableArea;
    Rectangle LeftFootRightKickableArea;
    Rectangle RightFootRightKickableArea;

    float m_footWidth;
    float m_ballRadius;
    bool m_pauseState;
    float m_variableGainValue;
    bool m_armCommandSent;
};

#endif

