#ifndef ROBOTCANDIDATE_H
#define ROBOTCANDIDATE_H

#include "Tools/Math/Vector2.h"

class RobotCandidate
{
public:
    Vector2<int> getTopLeft() const;
    Vector2<int> getBottomRight() const;

    int width();
    int height();
    float aspect();

    unsigned char getTeamColour() const;

    RobotCandidate();
    RobotCandidate(int left, int top, int right, int bottom);
    RobotCandidate(int left, int top, int right, int bottom, unsigned char teamColour);
    ~RobotCandidate();
private:
    Vector2<int> topLeft;
    Vector2<int> bottomRight;
    unsigned char teamColour;
};

#endif // ROBOTCANDIDATE_H
