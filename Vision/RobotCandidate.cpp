#include "RobotCandidate.h"

RobotCandidate::RobotCandidate()
{
    topLeft.x = 0;
    topLeft.y = 0;
    bottomRight.x = 0;
    bottomRight.y = 0;
}

RobotCandidate::RobotCandidate(int left, int top, int right, int bottom)
{
    topLeft.x = left;
    topLeft.y = top;
    bottomRight.x = right;
    bottomRight.y = bottom;
    //red = 4
    //blue = 9
    //unknown = 3
    teamColour = 3;
}

RobotCandidate::RobotCandidate(int left, int top, int right, int bottom, unsigned char teamColour):teamColour(teamColour)
{
    topLeft.x = left;
    topLeft.y = top;
    bottomRight.x = right;
    bottomRight.y = bottom;
}

RobotCandidate::~RobotCandidate()
{
    return;
}

Vector2<int> RobotCandidate::getTopLeft() const
{
    return topLeft;
}
Vector2<int> RobotCandidate::getBottomRight() const
{
    return bottomRight;
}

int RobotCandidate::width()
{
    return (bottomRight.x - topLeft.x);
}
int RobotCandidate::height()
{
    return (bottomRight.y - topLeft.y);
}
float RobotCandidate::aspect()
{
    return (float)(bottomRight.x - topLeft.x) / (float)(bottomRight.y - topLeft.y);
}

unsigned char RobotCandidate::getTeamColour() const
{
    return teamColour;
}


