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

#ifndef NAOKICK_H
#define NAOKICK_H

class NUSensorsData;
class NUActionatorsData;
class KickJob;
class NUWalk;
#include "Motion/NUKick.h"

#include "Kinematics/Kinematics.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include <string>

class FieldObjects;

class NAOKick : public NUKick
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

public:
    NAOKick(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions);
    ~NAOKick();

    void stopLegs();
    void kill();
    
    //bool isUsingHead();
    
    void loadKickParameters();

    std::string toString(swingDirection_t theSwingDirection);
    std::string toString(poseType_t thePose);
private:
    std::vector<float> bestKickingPosition(const std::vector<float>& ballPosition,const std::vector<float>& targetPositon);
    void kickToPoint(const std::vector<float>& position, const std::vector<float>& target);
    void preKick();
    void doKick();
    bool doPreKick();
    bool doPostKick();
    bool doPoise(KickingLeg leg, float angleChange, float speed);

    bool chooseLeg();
    bool kickAbortCondition();
    bool ShiftWeightToFoot(KickingLeg supportLeg, float targetWeightPercentage, float speed, float time);
    bool ShiftWeightToFootClosedLoop(KickingLeg supportLeg, float targetWeightPercentage, float speed);
    bool LiftKickingLeg(KickingLeg kickingLeg, float speed);
    bool SwingLegForward(KickingLeg kickingLeg, float speed);
    bool SwingLegSideward(KickingLeg kickingLeg, float speed);
    bool AlignYposition(KickingLeg kickingLeg, float speed, float yPos);
    bool AlignXposition(KickingLeg kickingLeg, float speed, float xPos);
    bool LowerLeg(KickingLeg kickingLeg, float speed);
    bool BalanceCoP(KickingLeg supportLeg, float targetX = 0.0f, float targetY = 0.0f);
    void BalanceCoPLevelTorso(KickingLeg theLeg, std::vector<float>& jointAngles, float CoPx, float CoPy, float targetX = 0.0f, float targetY = 0.0f);
    void BalanceCoPHipAndAnkle(std::vector<float>& jointAngles, float CoPx, float CoPy, float targetX = 0.0f, float targetY = 0.0f);
    void BalanceCoPHip(std::vector<float>& jointAngles, float CoPx, float CoPy = 0.0f);
    void BalanceCoPAnkle(std::vector<float>& jointAngles, float CoPx, float CoPy = 0.0f);
    void FlattenFoot(std::vector<float>& jointAngles);
    float FlatFootAnklePitch(float hipPitch, float kneePitch);
    float FlatFootAnkleRoll(float hipRoll);
    bool IsPastTime(float time);
    void MaintainSwingHeight(KickingLeg supportLeg, std::vector<float>& supportLegJoints, KickingLeg swingLeg, std::vector<float>& swingLegJoints, float swingHeight);
    double TimeBetweenFrames();
    float perSec2perFrame(float value);
    float SpeedMultiplier();
    float GainMultiplier();
    double MoveLimbToPositionWithSpeed(NUActionatorsData::id_t limbId, std::vector<float> currentPosition, std::vector<float> targetPosition, float maxSpeed , float gain, float smoothness = 0.5);
    double MoveLegsToPositionWithSpeed(const std::vector<float>& targetPosition, float maxSpeed , float gain, float smoothness = 0.5);

    float CalculateForwardSwingSpeed(float kickDistance);
    float CalculateSidewardSwingSpeed(float kickDistance);

    void MoveArmsToKickPose(KickingLeg leadingArmleg, float speed);

    Kinematics* m_kinematicModel;

    poseType_t pose;

    float m_defaultMotorGain;
    float m_defaultArmMotorGain;
    std::vector<float> m_leftLegInitialPose;
    std::vector<float> m_rightLegInitialPose;

    KickingLeg m_kickingLeg;
    swingDirection_t m_swingDirection;

    bool m_stateCommandGiven;
    double m_estimatedStateCompleteTime;

    class jointLimit
    {
    public:
        jointLimit(): min(0.0f), max(0.0f){};
        jointLimit(float minAngle, float maxAngle): min(minAngle), max(maxAngle){};
        float min;
        float max;
    };

    bool LimitJoints(KickingLeg leg, std::vector<float> jointPositions);
    std::vector<jointLimit> m_leftLegLimits;
    std::vector<jointLimit> m_rightLegLimits;
    Rectangle LeftFootForwardKickableArea;
    Rectangle RightFootForwardKickableArea;
    Rectangle LeftFootLeftKickableArea;
    Rectangle RightFootLeftKickableArea;
    Rectangle LeftFootRightKickableArea;
    Rectangle RightFootRightKickableArea;
    float m_intialWeightShiftPercentage;

    float m_footWidth;
    float m_ballRadius;
    bool m_pauseState;
    float m_variableGainValue;
    bool m_armCommandSent;
    bool m_kickWait;
};


#endif

