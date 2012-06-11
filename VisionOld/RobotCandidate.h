#ifndef ROBOTCANDIDATE_H
#define ROBOTCANDIDATE_H

#include "ObjectCandidate.h"
#include "Tools/Math/Vector2.h"
#include "Tools/Math/Line.h"

#include <vector>

class RobotCandidate: public ObjectCandidate
{
public:
    std::vector<Vector2<int> > getSkeleton() const;

    int width();
    int height();
    float aspect();

    unsigned char getTeamColour() const;

    RobotCandidate();
    RobotCandidate(int left, int top, int right, int bottom);
    RobotCandidate(int left, int top, int right, int bottom, std::vector<Vector2<int> > points);
    RobotCandidate(int left, int top, int right, int bottom, unsigned char teamColour);
    RobotCandidate(int left, int top, int right, int bottom, unsigned char teamColour, std::vector<Vector2<int> > points);
    ~RobotCandidate();
private:
    std::vector<Vector2<int> > skeleton;
};

#endif // ROBOTCANDIDATE_H
