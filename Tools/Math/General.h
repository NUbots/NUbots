/**
 * @file Math/General.h
 *
 */


#ifndef Math_General_h
#define Math_General_h

#include <cmath>

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
inline T max(T x, T y){
  if(x > y) return x;
  else return y;
}

template <class T>
inline T min(T x, T y){
  if(x < y) return x;
  else return y;
}

template <class T>
inline T norm(T theta){
    return atan2(sin(theta), cos(theta));
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

/**
* Round to the next integer
* @param d A number
* @return The number as integer
*/
inline int roundNumberToInt(double d)
{
  return static_cast<int>(floor(d+0.5));
}

} // End namespace

#endif
