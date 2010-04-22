/*! @file MotionCurves.h
    @brief Declaration of functions to calculate motion curves
 
    @class MotionCurves
    @brief A module to calculate smooth motion curves
 
    @author Jason Kulk
 
  Copyright (c) 2010 Jason Kulk
 
    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MOTIONCURVES_H
#define MOTIONCURVES_H

#include <vector>

class MotionCurves
{
public:
    static void calculate(double starttime, double stoptime, float startposition, float stopposition, float smoothness, int cycletime, std::vector<double>& calculatedtimes, std::vector<float>& calculatedpositions);
private:
    MotionCurves() {};
    ~MotionCurves() {};
    static void calculateTrapezoidalCurve(double starttime, double stoptime, float startposition, float stopposition, float startvelocity, float stopvelocity, float smoothness, int cycletime, std::vector<double>& calculatedtimes, std::vector<float>& calculatedpositions);
protected:
public:
};

#endif

