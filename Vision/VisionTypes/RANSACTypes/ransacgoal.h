#ifndef RANSACGOAL_H
#define RANSACGOAL_H

#include "Tools/Math/LSFittedLine.h"
#include "Vision/VisionTypes/coloursegment.h"

class RANSACGoal : public LSFittedLine
{
public:
    RANSACGoal();

    bool regenerate(const vector<ColourSegment> &segments);

    unsigned int minPointsForFit() const {return 2;}

    double calculateError(ColourSegment c) const;

    void fit(const vector<ColourSegment> &segments);

    double getWidth() const {return width_mean;}
    double getWidthUncertainty() const {return width_stddev;}

private:
    double width_mean,
           width_stddev;
};

#endif // RANSACGOAL_H
