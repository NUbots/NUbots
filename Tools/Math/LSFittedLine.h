#ifndef LSFITTEDLINE_H_DEFINED
#define LSFITTEDLINE_H_DEFINED
#include "Line.h"
#include <vector>
#include "Vector2.h"

using std::vector;

//class LinePoint: public Point{
//	public:
//		int ID;
//		bool inUse;
//		int width;
//		LinePoint();
//        LinePoint(double in_x, double in_y);
//		void clear();
//};

class LSFittedLine : public Line
{
  public:
    LSFittedLine();
    bool valid;
    
    void addPoint(Point &point);
    void addPoints(vector<Point>& pointlist);
    void joinLine(LSFittedLine &sourceLine);
    Vector2<double> combinedR2TLSandMSD(const LSFittedLine &sourceLine) const;
    double getMSD() const;
    double getr2tls() const;
    void clearPoints();
    const std::vector<Point>& getPoints();
    Point getLeftPoint() const;
    Point getRightPoint() const;
    int numPoints;
private:
    void calcLine();
    double sumX, sumY, sumX2, sumY2, sumXY;
    double MSD, r2tls;
    std::vector<Point> points;
    
};

#endif
