#include "Matrix.h"

namespace TransformMatrices
{
Matrix RotX(double angle);
Matrix RotY(double angle);
Matrix RotZ(double angle);
Matrix Translation(double dx, double dy, double dz);

struct DHParameters
{
    double alpha;
    double a;
    double thetaOffset;
    double d;
};

Matrix ModifiedDH(double alpha, double a, double theta, double d);
Matrix ModifiedDH(const DHParameters& paramteters, double theta);
}

