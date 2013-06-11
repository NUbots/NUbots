/**
 * @file Math/General.h
 *
 */


#ifndef MATH_GENERAL_H
#define MATH_GENERAL_H

#include <cmath>
#include <vector>
#include <iostream>
#include "Matrix.h"
#include "Vector3.h"


namespace mathGeneral
{

const double PI = 4.0*atan(1.0);

template <class T>
inline T sqr(T x){
  return x*x;
}

template <class T>
inline T abs(T x){
  if(x > 0) return x;
  else return -x;
}

template <class T>
inline T crop(T num, T low, T high){
  if (num < low) num = low;
  else if (num > high) num = high;
  return num;
}

template <class T>
inline int sign(T x){
  if(x < 0.0) return -1;
  else if(x > 0.0) return 1;
  else return 0;
}

inline double rad2deg(double x){
  return ((x)*180.0)/PI;
}

inline double deg2rad(double x){
  return ((x)*PI)/180.0;
}

inline double normaliseAngle(double theta){
    return atan2(sin(theta), cos(theta));
}

/**
* Round to the next integer
* @param d A number
* @return The number as integer
*/
inline int roundNumberToInt(double d)
{
  return static_cast<int>(floor(d+0.5));
}

inline std::vector<float> Spherical2Cartesian(const std::vector<float>& sphericalCoordinates)
{
    const float distance = sphericalCoordinates[0];
    const float bearingcos = cos(sphericalCoordinates[1]);
    const float bearingsin = sin(sphericalCoordinates[1]);
    const float elevationcos = cos(sphericalCoordinates[2]);
    const float elevationsin = sin(sphericalCoordinates[2]);

    std::vector<float> result(3,0.0f);
    result[0] = distance * bearingcos * elevationcos;
    result[1] = distance * bearingsin * elevationcos;
    result[2] = distance * elevationsin;
    return result;
}

template<typename T>
inline std::vector<T> Cartesian2Spherical(const std::vector<T>& cartesianCoordinates)
{
    const T x = cartesianCoordinates[0];
    const T y = cartesianCoordinates[1];
    const T z = cartesianCoordinates[2];
    std::vector<T> result(3,0.0f);

    result[0] = sqrt(x*x + y*y + z*z);
    result[1] = atan2(y,x);
    result[2] = asin(z/(result[0]));
    return result;
}

template<typename T>
inline Vector3<T> Spherical2Cartesian(const Vector3<T>& sphericalCoordinates)
{
    const T distance = sphericalCoordinates.x;
    const T bearingcos = cos(sphericalCoordinates.y);
    const T bearingsin = sin(sphericalCoordinates.y);
    const T elevationcos = cos(sphericalCoordinates.z);
    const T elevationsin = sin(sphericalCoordinates.z);

    Vector3<T> result;
    result.x = distance * bearingcos * elevationcos;
    result.y = distance * bearingsin * elevationcos;
    result.z = distance * elevationsin;
    return result;
}

template<typename T>
inline Vector3<T> Cartesian2Spherical(const Vector3<T>& cartesianCoordinates)
{
    const T x = cartesianCoordinates.x;
    const T y = cartesianCoordinates.y;
    const T z = cartesianCoordinates.z;
    Vector3<T> result;

    result.x = sqrt(x*x + y*y + z*z);
    result.y = atan2(y,x);
    result.z = asin(z/(result.x));
    return result;
}

inline Matrix Spherical2Cartesian(const Matrix& sphericalCoordinates)
{
    const float distance = sphericalCoordinates[0][0];
    const float bearingcos = cos(sphericalCoordinates[1][0]);
    const float bearingsin = sin(sphericalCoordinates[1][0]);
    const float elevationcos = cos(sphericalCoordinates[2][0]);
    const float elevationsin = sin(sphericalCoordinates[2][0]);

    Matrix result(3,1);
    result[0][0] = distance * bearingcos * elevationcos;
    result[1][0] = distance * bearingsin * elevationcos;
    result[2][0] = distance * elevationsin;
    return result;
}

inline Matrix Cartesian2Spherical(const Matrix& cartesianCoordinates)
{
    const float x = cartesianCoordinates[0][0];
    const float y = cartesianCoordinates[1][0];
    const float z = cartesianCoordinates[2][0];
    Matrix result(3,1);

    result[0][0] = sqrt(x*x + y*y + z*z);
    result[1][0] = atan2(y,x);
    result[2][0] = asin(z/(result[0][0]));
    return result;
}

/*! @brief Returns true if all of the values
    @param values the data to compare to zero
 */
template <typename T>
inline bool allZeros(const std::vector<T>& values) 
{
    bool result = true;
    for (size_t i=0; i<values.size(); i++)
        if (values[i] != 0)
            return false;
    return result;
}
    
/*! @brief Returns true if every element in left is within eps of every element in right
    @param left the left operand
    @param right the right operand
    @param eps the maximum difference allowed before elements are not equal
 */
template <typename T>
inline bool allEqual(const std::vector<T>& left, const std::vector<T>& right, T eps = 0.01)
{
    size_t n = std::min(left.size(), right.size());
    for (size_t i=0; i<n; i++)
        if (fabs(left[i] - right[i]) > eps)
            return false;
    return true;
}
    
/*! @brief Returns the magnitude of the maximum difference between elements in left and right
    @param left
    @param right
 */
template <typename T>
inline float maxDifference(const std::vector<T>& left, const std::vector<T>& right)
{
    size_t n = std::min(left.size(), right.size());
    float max = 0;
    for (size_t i=0; i<n; i++)
        if (fabs(left[i] - right[i]) > max)
            max = fabs(left[i] - right[i]);
    return max;
}

/*! @brief Determines whether the point is inside the convex hull specifed by vertices.
    @param x the x value of the point to test
    @param y the y value of the point to test
    @param vertices the [[x0,y0], [x1,y1], ... , [xn,yn]] specifying the convex hull. 
           If x is up, y is left (ie the NUbot convention), then the vertices need to be in clockwise order.
           If x is up, y is right, then the vertices need to be in anti-clockwise order
    @param margin the amount to 'shrink' the convex hull and then test if the point is inside it
 	@return true if [x,y] is inside the hull specified by vertices
 */
inline bool PointInsideConvexHull(float x, float y, const std::vector<std::vector<float> >& vertices, float margin = 0)
{
    if (x > 0)
        x += margin;
    else
        x -= margin;
    if (y > 0)
        y += margin;
    else
        y -= margin;
    
    size_t n = vertices.size();
    for (size_t i=0; i<n; i++)
    {
        if ((vertices[(i+1)%n][0] - x)*(vertices[i][1] - y) - (vertices[i][0] - x)*(vertices[(i+1)%n][1] - y) < 0)
            return false;
    }
    return true;
}

/*! @brief Assigns C to the projected point from A through B to distancePast after B.
    @param A the start point
    @param B the point to project through
    @param distancePast the distance past B to go
    @param C the destination value
 */
inline void ProjectFromAtoB(float* A, float* B, float distancePast, float* C) {
    float xdiff = B[0]-A[0];
    float ydiff = B[1]-A[1];
    float dist = sqrt(sqr(xdiff)+sqr(ydiff));
    dist = (dist+distancePast)/dist;
    C[0] = A[0]+dist*xdiff;
    C[1] = A[1]+dist*ydiff;
}


//fast (approximate) inverse square root. Quite accurate.
inline double invSqrt( const double& x )
{
    double y = x;
    double xhalf = ( double )0.5 * y;
    long long i = *( long long* )( &y );
    i = 0x5fe6ec85e7de30daLL - ( i >> 1 );//LL suffix for (long long) type for GCC
    y = *( double* )( &i );
    y = y * ( ( double )1.5 - xhalf * y * y );
    
    return y;
}

//fast (approximate) inverse square root. Quite accurate.
inline float invSqrt( const float& number )
{
       long i;
       float x2, y;
       const float threehalfs = 1.5f;

       x2 = number * 0.5f;
       y = number;
       i = * ( long * ) &y; // evil floating point bit level hacking
       i = 0x5f3759df - ( i >> 1 ); // what the fuck?
       y = * ( float * ) &i;
       y = y * ( threehalfs - ( x2 * y * y ) ); // 1st iteration
       // y = y * ( threehalfs - ( x2 * y * y ) ); // 2nd iteration, this can be removed

       return y;
} 

//fast (approximate) inverse square root. Up to 6% error.
inline double fSqrt(const double& x) {
   unsigned long long i = *(unsigned long long*) &x; 
   // adjust bias
   i  += (( long long)1023) << ((long long)52);
   // approximation of square root
   i >>= 1; 
   return *(double*) &i;
 }

//fast (approximate) inverse square root. Up to 6% error.
inline float fSqrt(const float& x) {
   unsigned int i = *(unsigned int*) &x; 
   // adjust bias
   i  += 127 << 23;
   // approximation of square root
   i >>= 1; 
   return *(float*) &i;
 }

} // End namespace

#endif //MATH_GENERAL_H
