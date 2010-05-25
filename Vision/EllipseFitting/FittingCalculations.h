#include "tnt.h"
#include "jama_eig.h"


class FittingCalculations{
	public:
                FittingCalculations();
                Array2D<double> Inverse33(Array2D<double> A);
                Array2D<double> Inverse22(Array2D<double> a);
                Array2D<double> Eig(Array2D<double> a);
                Array2D<double> transp(Array2D<double> a);
                void SolveEllipse(Array1D<double> a);
                void Print(Array2D<double> a);

                double cx;
                double cy;
                double theta;
                double r1;
                double r2;
};
