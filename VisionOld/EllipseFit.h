
#include "../Tools/Math/LSFittedLine.h"
#include <vector>

class EllipseFit{
    public:
        EllipseFit();
        ~EllipseFit();
        void Fit_Ellipse( std::vector < LinePoint* > centreCirclePoints);
        double GetX();
        double GetY();
        double GetR1();
        double GetR2();
        double GetTheta();
        void PrintFinal();
    private:
        double cx;
        double cy;
        double r1;
        double r2;
        double theta;

        
};
