#ifndef ROBOCUPHACKS_H
#define ROBOCUPHACKS_H

#include "Vision/VisionTypes/VisionFieldObjects/ball.h"
#include "Vision/VisionTypes/VisionFieldObjects/beacon.h"
#include "Vision/VisionTypes/VisionFieldObjects/goal.h"
#include "Vision/VisionTypes/quad.h"

class RobocupHacks
{
public:
    static void ballGoalHack();

private:
    RobocupHacks();

    static bool ballInQuad(const Ball& ball, const Quad& quad);
};

#endif // ROBOCUPHACKS_H
