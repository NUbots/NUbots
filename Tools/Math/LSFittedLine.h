#ifndef LSFITTEDLINE_H_DEFINED
#define LSFITTEDLINE_H_DEFINED
#include "Line.h"
#include <vector>

class LinePoint: public Point{
	public:
		int ID;
		bool inUse;
		int width;
		LinePoint();
		void clear();
};

class LSFittedLine : public Line
{
  public:
    LSFittedLine();
    bool valid;
    
    void addPoint(LinePoint &point);
    void joinLine(LSFittedLine &sourceLine);
    double getMSD();
    double getr2tls();
    Point leftPoint, rightPoint;
    void clearPoints();
    std::vector<LinePoint*> getPoints();
    int numPoints;
private:
    void calcLine();
    double sumX, sumY, sumX2, sumY2, sumXY;
    double MSD, r2tls;
    std::vector<LinePoint*> points;
    
};

#endif
