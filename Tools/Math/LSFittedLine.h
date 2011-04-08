#ifndef LSFITTEDLINE_H_DEFINED
#define LSFITTEDLINE_H_DEFINED
#include "Line.h"
#include <vector>
#include "Vector2.h"

using std::vector;

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
    void addPoints(vector<LinePoint*>& pointlist);
    void joinLine(LSFittedLine &sourceLine);
    Vector2<double> combinedR2TLSandMSD(const LSFittedLine &sourceLine) const;
    double getMSD() const;
    double getr2tls() const;
    Point leftPoint, rightPoint;
    Point transLeftPoint, transRightPoint;
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
