
#include "EllipseFitting/tnt.h"
#include "EllipseFitting/jama_eig.h"

class EllipseFit{
    public:
        EllipseFit();
        ~EllipseFit();
        void Fit_Ellipse(Array2D<double> x, Array2D<double> y);
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

        Array2D<double> Inverse33(Array2D<double> A);
        Array2D<double> Inverse22(Array2D<double> a);
        Array2D<double> Eig(Array2D<double> a);
        Array2D<double> transp(Array2D<double> a);
        void SolveEllipse(Array1D<double> a);
        void Print(Array2D<double> a);
        
};
