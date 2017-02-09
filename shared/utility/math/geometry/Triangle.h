#ifndef UTILITY_MATH_GEOMETRY_TRIANGLE_H
#define UTILITY_MATH_GEOMETRY_TRIANGLE_H

#include <armadillo>

#include "ros/ros.h"

#include "utility/armadillo/armadillo.h"

namespace utility
{
    namespace math
    {
        namespace geometry
        {
            class Triangle
            {
            private:
                arma::vec3 P0, P1, P2, normal;
                double     epsilon, area;

            public:
                Triangle() : P0(arma::fill::zeros), P1(arma::fill::zeros), P2(arma::fill::zeros), epsilon(1e-6)
                {
                }

                Triangle(const arma::vec3& P0, const arma::vec3& P1, const arma::vec3& P2, double epsilon = 1e-6)
                {
                    this->P0      = P0;
                    this->P1      = P1;
                    this->P2      = P2;
                    this->epsilon = epsilon;
                    calculateNormal();
                    calculateArea();
                }

                Triangle(const arma::vec3& normal, const arma::vec3& P0, const arma::vec3& P1, const arma::vec3& P2, double epsilon = 1e-6)
                {
                    this->normal  = normal;
                    this->P0      = P0;
                    this->P1      = P1;
                    this->P2      = P2;
                    this->epsilon = epsilon;
                    calculateArea();
                }

                arma::vec3 calculateNormal(bool CCW = true)
                {
                    if (CCW == true)
                    {
                        normal = arma::normalise(arma::cross(P1 - P0, P2 - P0));
                    }

                    else
                    {
                        normal = arma::normalise(arma::cross(P2 - P0, P1 - P0));
                    }

                    return(normal);
                }

                double calculateArea()
                {
                    area = 0.5 * std::abs(arma::norm(arma::cross(P1 - P0, P2 - P0)));
                    return(area);
                }

                double getArea() const
                {
                    return(area);
                }

                arma::vec3 getNormal() const
                {
                    return(normal);
                }

                arma::vec3 getP0() const
                {
                    return(P0);
                }

                arma::vec3 getP1() const
                {
                    return(P1);
                }

                arma::vec3 getP2() const
                {
                    return(P2);
                }

                void applyTransform(const arma::mat44& transform)
                {
                    arma::vec4 norm(arma::fill::zeros), R0(arma::fill::ones), R1(arma::fill::ones), R2(arma::fill::ones);
                    norm.head(3) = normal;
                    R0.head(3)   = P0;
                    R1.head(3)   = P1;
                    R2.head(3)   = P2;

                    norm = arma::normalise(transform * norm);
                    R0   = transform * R0;
                    R1   = transform * R1;
                    R2   = transform * R2;

                    normal = normal.head(3);
                    P0     = R0.head(3);
                    P1     = R1.head(3);
                    P2     = R2.head(3);
                }

                arma::vec3 getRandomPoint() const
                {
                    // Randomly select a point from inside a triangle.
                    // https://wiki.csiro.au/display/AutonomousSystems/Fitting+CAD+models+in+a+Point+Cloud
                    //      ----> Papers
                    //            ----> Graphics Gems I
                    //                  ----> Method 1
                    const double SCALE = (1.0 / (RAND_MAX + 1.0));

                    double sqrtT = std::sqrt(rand() * SCALE);
                    double s     = rand() * SCALE;

                    arma::vec3 scale = {(1 - sqrtT), ((1 - s) * sqrtT), (s * sqrtT)};

                    arma::mat33 points;
                    points.col(0) = P0;
                    points.col(1) = P2;
                    points.col(2) = P2;

                    return(points * scale);
                }

                // Taken from
                // https://github.com/erich666/jgt-code/blob/master/Volume_02/Number_1/Moller1997a/raytri.cm
                bool rayIntersect(const arma::vec3& orig, const arma::vec3& dir)
                {
                    return(rayIntersect(orig, dir, P0, P1, P2, epsilon));
                }

                static bool rayIntersect(const arma::vec3& orig, const arma::vec3& dir, const arma::mat33& points,
                                         double epsilon = 1e-6)
                {
                    return(rayIntersect(orig, dir, points.col(0), points.col(1), points.col(2), epsilon));
                }

                // Taken from
                // https://github.com/erich666/jgt-code/blob/master/Volume_02/Number_1/Moller1997a/raytri.cm
                static bool rayIntersect(const arma::vec3& orig, const arma::vec3& dir,
                                         const arma::vec3& P0, const arma::vec3& P1, const arma::vec3& P2,
                                         double epsilon = 1e-6)
                {
                    /* find vectors for two edges sharing vert0 */
                    arma::vec3 edge1 = P1 - P0;
                    arma::vec3 edge2 = P2 - P0;

                    /* begin calculating determinant - also used to calculate U parameter */
                    arma::vec3 pvec = arma::cross(dir, edge2);

                    /* if determinant is near zero, ray lies in plane of triangle */
                    double det = arma::dot(edge1, pvec);

                    if (std::abs(det) < epsilon)
                    {
                        return(false);
                    }

                    double inv_det = 1.0 / det;

                    /* calculate distance from vert0 to ray origin */
                    arma::vec3 tvec = orig - P0;

                    /* calculate U parameter and test bounds */
                    double u = arma::dot(tvec, pvec) * inv_det;

                    if ((u < 0.0) || (u > 1.0))
                    {
                        return(false);
                    }

                    /* prepare to test V parameter */
                    arma::vec3 qvec = arma::cross(tvec, edge1);

                    /* calculate V parameter and test bounds */
                    double v = arma::dot(dir, qvec) * inv_det;

                    if ((v < 0.0) || ((u + v) > 1.0))
                    {
                        return(false);
                    }

                    double t = arma::dot(edge2, qvec) * inv_det;

                    return(t > epsilon);
                }
            };
        }
    }
}

#endif // UTILITY_MATH_GEOMETRY_TRIANGLE_H
