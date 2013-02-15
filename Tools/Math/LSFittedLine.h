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
    LSFittedLine(vector<Point>& pointlist);
    ~LSFittedLine();
    bool valid;
    
    void addPoint(const Point &point);
    void addPoints(const vector<Point>& pointlist);
    void joinLine(LSFittedLine &sourceLine);
    Vector2<double> combinedR2TLSandMSD(const LSFittedLine &sourceLine) const;
    double getMSD() const;
    double getr2tls() const;
    void clearPoints();
    unsigned int getNumPoints() const {return points.size();}
    const std::vector<Point>& getPoints() const;
    bool getEndPoints(Vector2<Point>& endpts) const;
    double averageDistanceBetween(const LSFittedLine& other) const;
private:
    void calcLine();
    double sumX, sumY, sumX2, sumY2, sumXY;
    double MSD, r2tls;
    std::vector<Point> points;
    
};

#endif
