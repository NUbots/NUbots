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

void RobocupHacks::beaconGoalHack()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    vector<Goal>& goals = vbb->getGoals();
    vector<Beacon>& beacons = vbb->getBeacons();
    vector<Goal>::iterator g_it = goals.begin();
    vector<Beacon>::iterator b_it;

    while(g_it < goals.end()) {
        bool thrown = false;
        b_it = beacons.begin();
        while(b_it < beacons.end()) {
            if(beaconAndGoalClose(*b_it, *g_it)) {
                //throw out goal and set beacon to ambiguous
                g_it = goals.erase(g_it);
                b_it->setUnknown();
                thrown = true;
                break;
            }
            b_it++;
        }
        if(!thrown) {
            g_it++;
        }
        else {
            #ifdef DEBUG_VISION_VERBOSITY_ON
                debug << "RobocupHacks::beaconGoalHack - goal thrown out: too near beacon" << endl;
            #endif
        }
    }
}

void RobocupHacks::ballGoalHack()
{
    VisionBlackboard* vbb = VisionBlackboard::getInstance();
    vector<Goal>& goals = vbb->getGoals();
    vector<Beacon>& beacons = vbb->getBeacons();
    vector<Ball>& balls = vbb->getBalls();
    vector<Goal>::iterator g_it;
    vector<Beacon>::iterator be_it;
    vector<Ball>::iterator ba_it = balls.begin();

    while(ba_it < balls.end()) {
        bool thrown = false;
        //check in beacons
        be_it = beacons.begin();
        while(be_it < beacons.end()) {
            if(ballInQuad(*ba_it, be_it->getQuad())) {
                //throw out ball
                ba_it = balls.erase(ba_it);
                thrown = true;
                break;
            }
            be_it++;
        }
        if(!thrown) {
            //check in goals
            g_it = goals.begin();
            while(g_it < goals.end()) {
                if(ballInQuad(*ba_it, g_it->getQuad())) {
                    //throw out ball
                    ba_it = balls.erase(ba_it);
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
                    debug << "RobocupHacks::ballGoalHack - ball thrown out in beacon or goal" << endl;
                #endif
            }
        }
    }
}

bool RobocupHacks::beaconAndGoalClose(const Beacon& beacon, const Goal& goal)
{
    return abs(beacon.getQuad().getBottomCentre().x - goal.getQuad().getBottomCentre().x) < 20;
}

bool RobocupHacks::ballInQuad(const Ball& ball, const Quad& quad)
{
    return ball.getLocationPixels().y < quad.getBottomLeft().y and
            ball.getLocationPixels().x >= quad.getBottomLeft().x and
            ball.getLocationPixels().x <= quad.getTopRight().x;
}

