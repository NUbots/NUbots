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
    LSFittedLine(const std::vector< Vector2<double> >& pointlist);
    virtual ~LSFittedLine();
    bool valid;
    
    void addPoint(const Vector2<double> &point);
    void addPoints(const std::vector< Vector2<double> >& pointlist);
    void joinLine(const LSFittedLine &sourceLine);
    Vector2<double> combinedR2TLSandMSD(const LSFittedLine &sourceLine) const;
    double getMSD() const;
    double getr2tls() const;
    void clearPoints();
    unsigned int getNumPoints() const {return points.size();}
    const std::vector< Vector2<double> >& getPoints() const;
    bool getEndPoints(Vector2<double>& p1, Vector2<double>& p2) const;
    bool getOriginalEndPoints(Vector2<double>& p1, Vector2<double>& p2) const;
    double averageDistanceBetween(const LSFittedLine& other) const;
private:
    void calcLine();
    double sumX, sumY, sumX2, sumY2, sumXY;
    double MSD, r2tls;
    std::vector< Vector2<double> > points;
    
};

#endif
