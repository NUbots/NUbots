#ifndef IK_SQUAT_H
#define IK_SQUAT_H

#include "Ik_motion.h"
#include "Tools/Math/TransformMatrices.h"
#include <iostream>

class Ik_squat: public Ik_motion
{
public:
    Ik_squat():Ik_motion(){}
    void initialiseSquat(float max_height, float min_height, float foot_angle, float stance_width)
    {
        m_max_height = max_height;
        m_min_height = min_height;
        m_foot_angle = foot_angle;
        m_stance_width = stance_width;
    }

    // Functions to get pose for each of the limbs.
    Matrix headPose()
    {
        return Matrix();
    }
    Matrix leftFootPose()
    {
        return footPose(true);
    }
    Matrix rightFootPose()
    {
        return footPose(false);
    }
    Matrix leftHandPose()
    {
        return Matrix();
    }
    Matrix rightHandPose()
    {
        return Matrix();
    }

protected:
    float m_max_height;
    float m_min_height;
    float m_foot_angle;
    float m_stance_width;

    float currentHeight()
    {
        std::cout << "getting height..." << std::endl;
        float prog_time = m_current_time_ms - m_cycle_start_time_ms;
        float progress = prog_time / m_period_ms;
        std::cout << "m_current_time_ms: " << m_current_time_ms << std::endl;
        std::cout << "m_cycle_start_time_ms: " << m_cycle_start_time_ms << std::endl;
        std::cout << "m_period_ms: " << m_period_ms << std::endl;
        std::cout << "progress: " << progress << std::endl;
        // using linear interpolation
        float start_height = 0;
        float end_height = 0;
        // going down
        if(progress <= 0.5)
        {
            progress *= 2;
            start_height = m_max_height;
            end_height = m_min_height;
        }
        // going back up
        else
        {
            progress = 2*(progress - 0.5);
            start_height = m_min_height;
            end_height = m_max_height;
        }
        float height = start_height + progress * (end_height - start_height);
        return height;
    }

    Matrix footPose(bool isLeft)
    {
        const float mult = isLeft?0.5:-0.5;
        const float half_stance = m_stance_width * mult;
        const float half_angle = m_foot_angle * mult;
        Matrix pose;
        float height = currentHeight();
        std::cout << "making pose..." << std::endl;
        pose = TransformMatrices::Translation(0.0, half_stance, height) * TransformMatrices::RotZ(half_angle);
        return pose;
    }
};

#endif // IK_MOTION_H
