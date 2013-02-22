#ifndef RANSACGOAL_H
#define RANSACGOAL_H

#include "Tools/Math/LSFittedLine.h"
//#include "Tools/Math/Line.h"
#include "Vision/VisionTypes/coloursegment.h"

//class RANSACGoal : public LSFittedLine
class RANSACGoal
{
public:
    RANSACGoal();

    bool regenerate(const vector<ColourSegment> &segments);

    unsigned int minPointsForFit() const {return 2;}

    double calculateError(ColourSegment c) const;

    double getInterpolatedWidth(Point p) const;

    LSFittedLine getLine() const { return l; }

    std::pair<Point, Point> getEndPoints() const { return std::pair<Point, Point>(s1, s2); }
    std::pair<double, double> getWidths() const { return std::pair<double, double>(w1, w2); }
    Vector2<double> getDirection() const { return v; }
    void fit(const vector<ColourSegment> &segments);

//    void merge(const RANSACGoal& other);

//    double getWidth() const {return width_mean;}
//    double getWidthUncertainty() const {return width_stddev;}

private:
    Point s1, s2;
    LSFittedLine l;
    double w1, w2, width_diff;
    Vector2<double> v;
    double len;
//    double width_mean,
//           width_stddev;
};

#endif // RANSACGOAL_H
