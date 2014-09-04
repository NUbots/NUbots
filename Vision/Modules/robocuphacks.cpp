#include "robocuphacks.h"
#include "Vision/visionblackboard.h"
#include "Vision/VisionTypes/VisionFieldObjects/ball.h"
#include "Vision/VisionTypes/VisionFieldObjects/beacon.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "debug.h"
#include "debugverbosityvision.h"

RobocupHacks::RobocupHacks()
{
}

void RobocupHacks::ballGoalHack()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    std::vector<Goal>::iterator g_it;
    std::vector<Ball>::iterator ba_it = vbb->m_balls.begin();

    while(ba_it < vbb->m_balls.end()) {
        bool thrown = false;
        //check in goals
        g_it = vbb->m_goals.begin();
        while(g_it < vbb->m_goals.end()) {
            if(ballInQuad(*ba_it, g_it->getQuad())) {
                //throw out ball
                ba_it = vbb->m_balls.erase(ba_it);
                thrown = true;
                break;
            }
            g_it++;
        }
        if(!thrown) {
            ba_it++;
        }
        else {
            #ifdef DEBUG_VISION_VERBOSITY_ON
                debug << "RobocupHacks::ballGoalHack - ball thrown out in beacon or goal" << std::endl;
            #endif
        }
    }
}

bool RobocupHacks::ballInQuad(const Ball& ball, const Quad& quad)
{
    return ball.getLocationPixels().y < quad.getBottomLeft().y and
            ball.getLocationPixels().x >= quad.getBottomLeft().x and
            ball.getLocationPixels().x <= quad.getTopRight().x;
}

