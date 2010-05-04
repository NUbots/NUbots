/*! @file MotionCurves.cpp
    @brief Implementation of motion curve calculation class

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

#include "MotionCurves.h"
#include "debug.h"
#include "debugverbositynumotion.h"

#include <math.h>

using namespace std;

/*! @brief Calculates a smooth motion curve for a single joint to a single position
    @param starttime the time in ms to start moving to the given position
    @param stoptime the time in ms to reach the given position
    @param startposition the start postion for the curve
    @param stopposition the stop position for the curve
    @param smoothness a fraction indicating the smoothness of the motion: 0 means linear motion curve, 1 minimises the acceleration and jerk
    @param cycletime the motion cycle time in ms. This is used to decide how many points to generate
    @param calculatedtimes the calculated times for the curve. Do not assume the times will be evenly spaced!
    @param calculatedpositions the calculated positions for the curve.
    @param calculatedvelocities the calculated velocities for the curve
 */
void MotionCurves::calculate(double starttime, double stoptime, float startposition, float stopposition, float smoothness, int cycletime, vector<double>& calculatedtimes, vector<float>& calculatedpositions, vector<float>& calculatedvelocities)
{
    calculateTrapezoidalCurve(starttime, stoptime, startposition, stopposition, 0, 0, smoothness, cycletime, calculatedtimes, calculatedpositions, calculatedvelocities);
}

/*! @brief Calculates a smooth motion curve for a single joint
    @param starttime the time in ms to start moving
    @param times the times in ms to reach the given positions [time0, time1, ... , timeN]
    @param startposition the start postion for the curve
    @param positions the target positions for the curve [position0, position1, ... positionN]
    @param smoothness a fraction indicating the smoothness of the motion: 0 means linear motion curve, 1 minimises the acceleration and jerk
    @param cycletime the motion cycle time in ms. This is used to decide how many points to generate
    @param calculatedtimes the calculated times for the curve. Do not assume the times will be evenly spaced!
    @param calculatedpositions the calculated positions for the curve.
    @param calculatedvelocities the calculated velocities for the curve
 */
void MotionCurves::calculate(double starttime, const vector<double>& times, float startposition, const vector<float>& positions, float smoothness, int cycletime, vector<double>& calculatedtimes, vector<float>& calculatedpositions, vector<float>& calculatedvelocities)
{
    if (times.empty())
        return;
    else if (positions.size() < times.size())
    {
        errorlog << "MotionCurves::calculate() failed because times.size(): " << times.size() << " positions.size(): " << positions.size() << endl;
        return;
    }
    else if (times.size() == 1)
        calculateTrapezoidalCurve(starttime, times[0], startposition, positions[0], 0, 0, smoothness, cycletime, calculatedtimes, calculatedpositions, calculatedvelocities);
    else
    {   // size is two or more, so I need to match the velocities between each pair
        float finalvelocity;
        finalvelocity = calculateFinalVelocity(starttime, times[0], times[1], startposition, positions[0], positions[1]); 
        calculateTrapezoidalCurve(starttime, times[0], startposition, positions[0], 0, finalvelocity, smoothness, cycletime, calculatedtimes, calculatedpositions, calculatedvelocities);
        vector<double> temptimes;
        vector<float> temppositions;
        vector<float> tempvelocities;
        for (unsigned int i=1; i<times.size()-1; i++)
        {
            finalvelocity = calculateFinalVelocity(times[i-1], times[i], times[i+1], positions[i-1], positions[i], positions[i+1]);
            calculateTrapezoidalCurve(times[i-1], times[i], positions[i-1], positions[i], calculatedvelocities.back(), finalvelocity, smoothness, cycletime, temptimes, temppositions, tempvelocities);
            calculatedtimes.insert(calculatedtimes.end(), temptimes.begin(), temptimes.end());
            calculatedpositions.insert(calculatedpositions.end(), temppositions.begin(), temppositions.end());
            calculatedvelocities.insert(calculatedvelocities.end(), tempvelocities.begin(), tempvelocities.end());
        }
        calculateTrapezoidalCurve(calculatedtimes.back(), times.back(), calculatedpositions.back(), positions.back(), calculatedvelocities.back(), 0, smoothness, cycletime, temptimes, temppositions, tempvelocities);
        calculatedtimes.insert(calculatedtimes.end(), temptimes.begin(), temptimes.end());
        calculatedpositions.insert(calculatedpositions.end(), temppositions.begin(), temppositions.end());
        calculatedvelocities.insert(calculatedvelocities.end(), tempvelocities.begin(), tempvelocities.end());
    }
}

/*! @brief Calculates a smooth motion curve for a single joint
    @param starttime the time in ms to start moving
    @param times the times in ms to reach the given positions [time0, time1, ... , timeN]
    @param startposition the start postion for the curve
    @param positions the target positions for the curve [position0, position1, ... positionN]
    @param gains the target gains for the curve [gain0, gain1, ... gainN]
    @param smoothness a fraction indicating the smoothness of the motion: 0 means linear motion curve, 1 minimises the acceleration and jerk
    @param cycletime the motion cycle time in ms. This is used to decide how many points to generate
    @param calculatedtimes the calculated times for the curve. Do not assume the times will be evenly spaced!
    @param calculatedpositions the calculated positions for the curve.
    @param calculatedvelocities the calculated velocities for the curve
    @param calculatedgains the calculated gains for the curve
 */
void MotionCurves::calculate(double starttime, const vector<double>& times, float startposition, const vector<float>& positions, const vector<float>& gains, float smoothness, int cycletime, vector<double>& calculatedtimes, vector<float>& calculatedpositions, vector<float>& calculatedvelocities, vector<float>& calculatedgains)
{
    if (times.empty())
        return;
    else if (positions.size() < times.size())
    {
        errorlog << "MotionCurves::calculate() failed because times.size(): " << times.size() << " positions.size(): " << positions.size() << endl;
        return;
    }
    else if (times.size() == 1)
        calculateTrapezoidalCurve(starttime, times[0], startposition, positions[0], 0, 0, smoothness, cycletime, calculatedtimes, calculatedpositions, calculatedvelocities);
    else
    {
        float finalvelocity;
        finalvelocity = calculateFinalVelocity(starttime, times[0], times[1], startposition, positions[0], positions[1]);
        calculateTrapezoidalCurve(starttime, times[0], startposition, positions[0], 0, finalvelocity, smoothness, cycletime, calculatedtimes, calculatedpositions, calculatedvelocities);
        calculatedgains = vector<float>(calculatedtimes.size(), gains[0]);
        vector<double> temptimes;
        vector<float> temppositions;
        vector<float> tempvelocities;
        vector<float> tempgains;
        for (unsigned int i=1; i<times.size()-1; i++)
        {
            finalvelocity = calculateFinalVelocity(times[i-1], times[i], times[i+1], positions[i-1], positions[i], positions[i+1]);
            calculateTrapezoidalCurve(times[i-1], times[i], positions[i-1], positions[i], calculatedvelocities.back(), finalvelocity, smoothness, cycletime, temptimes, temppositions, tempvelocities);
            tempgains = vector<float> (temptimes.size(), gains[i]);
            calculatedtimes.insert(calculatedtimes.end(), temptimes.begin(), temptimes.end());
            calculatedpositions.insert(calculatedpositions.end(), temppositions.begin(), temppositions.end());
            calculatedvelocities.insert(calculatedvelocities.end(), tempvelocities.begin(), tempvelocities.end());
            calculatedgains.insert(calculatedgains.end(), tempgains.begin(), tempgains.end());
        }
        calculateTrapezoidalCurve(calculatedtimes.back(), times.back(), calculatedpositions.back(), positions.back(), calculatedvelocities.back(), 0, smoothness, cycletime, temptimes, temppositions, tempvelocities);
        tempgains = vector<float> (temptimes.size(), gains.back());
        calculatedtimes.insert(calculatedtimes.end(), temptimes.begin(), temptimes.end());
        calculatedpositions.insert(calculatedpositions.end(), temppositions.begin(), temppositions.end());
        calculatedvelocities.insert(calculatedvelocities.end(), tempvelocities.begin(), tempvelocities.end());
        calculatedgains.insert(calculatedgains.end(), tempgains.begin(), tempgains.end());
    }
}

/*! @brief Calculates a smooth motion curve for a several joints. Note the input order is a little unusual for this function
 
    The data needs to be of the following format:
    [time0, time1, ... , timeN] and [[position0, ..., positionM]_1, [position0, ..., positionM]_2, ... , [position0, ..., positionM]_N]
    where N is the number of time points, and M is the number of joints
 
    Note that the calculated times and positions will /b NOT have the same format. They need to have the following format:
    [[time0, time1, ...]_0, [time0, time1, ...]_1, ..., [time0, time1, ...]]_M
    [[posi0, posi1, ...]_0, [posi0, posi1, ...]_1, ..., [posi0, posi1, ...]]_M
    where M is the number of joints.
 
    Because of this format conversion this function has a little overhead.
 
    @param starttime the time in ms to start moving
    @param times the times in ms to reach the given positions [time0, time1, ... , timeN]
    @param startpositions the start postion for each joint [start0, start1, ... , startM]
    @param positions the target positions for the curve [[position0, ..., positionM]_0, [position1, ..., positionM]_1, ... , [position1, ..., positionM]_N]
    @param smoothness a fraction indicating the smoothness of the motion: 0 means linear motion curve, 1 minimises the acceleration and jerk
    @param cycletime the motion cycle time in ms. This is used to decide how many points to generate
    @param calculatedtimes the calculated times for the curve. Do not assume the times will be evenly spaced! [[time0, time1, ...]_0, [time0, time1, ...]_1, ..., [time0, time1, ...]]_M
    @param calculatedpositions the calculated positions for the curve. [[posi0, posi1, ...]_0, [posi0, posi1, ...]_1, ..., [posi0, posi1, ...]]_M
    @param calculatedvelocities the calculated velocities for the curve. [[velo0, velo1, ...]_0, [velo0, velo1, ...]_1, ..., [velo0, velo1, ...]]_M
 */
void MotionCurves::calculate(double starttime, const vector<double>& times, const vector<float>& startpositions, const vector<vector<float> >& positions, float smoothness, int cycletime, vector<vector<double> >& calculatedtimes, vector<vector<float> >& calculatedpositions, vector<vector<float> >& calculatedvelocities)
{
    if (times.empty())
        return;
    else if (positions.size() < times.size())
    {
        errorlog << "MotionCurves::calculate() failed because times.size(): " << times.size() << " positions.size(): " << positions.size() << " startpositions.size(): " << startpositions.size() << endl;
        return;
    }
    
    // as discussed above I need to reorder the positions
    vector<vector<float> > reorderedpositions(positions[0].size(), vector<float>(positions.size(), 0));     // positions.size() = number of time points and positions[0].size() = number of joints
    for (unsigned int i=0; i<positions.size(); i++)
    {
        for (unsigned int j=0; j<positions[i].size(); j++)
            reorderedpositions[j][i] = positions[i][j];
    }
    unsigned int numjoints = startpositions.size();
    calculatedtimes = vector<vector<double> >();
    calculatedtimes.reserve(numjoints);
    calculatedpositions = vector<vector<float> >();
    calculatedpositions.reserve(numjoints);
    calculatedvelocities = vector<vector<float> >();
    calculatedvelocities.reserve(numjoints);
    for (unsigned int i=0; i<numjoints; i++)
    {
        vector<double> temptimes;
        vector<float> temppositions;
        vector<float> tempvelocities;
        calculate(starttime, times, startpositions[i], reorderedpositions[i], smoothness, cycletime, temptimes, temppositions, tempvelocities);
        calculatedtimes.push_back(temptimes);
        calculatedpositions.push_back(temppositions);
        calculatedvelocities.push_back(tempvelocities);
    }
}

/*! @brief Calculates a smooth motion curve for several joints. Each joint has its own time vector
 
    The data needs to be of the following format:
    [[time0, time1, ..., timeI]_0, [time0, time1, ..., timeJ]_1, ... , [time0, time1, ..., posiZ]_N]
    [[posi0, posi1, ..., posiI]_0, [posi0, posi1, ..., posiJ]_1, ... , [posi0, posi1, ..., posiZ]_N]
    where each time-position pair has the same length, and there are N joints
 
    @param starttime the time in ms to start moving
    @param times the times in ms to reach the given positions [time0, time1, ... , timeN]
    @param startpositions the start postion for each joint [start0, start1, ... , startM]
    @param positions the target positions for the curve [[position0, ..., positionM]_0, [position1, ..., positionM]_1, ... , [position1, ..., positionM]_N]
    @param smoothness a fraction indicating the smoothness of the motion: 0 means linear motion curve, 1 minimises the acceleration and jerk
    @param cycletime the motion cycle time in ms. This is used to decide how many points to generate
    @param calculatedtimes the calculated times for the curve. Do not assume the times will be evenly spaced! [[time0, time1, ...]_0, [time0, time1, ...]_1, ..., [time0, time1, ...]]_M
    @param calculatedpositions the calculated positions for the curve. [[posi0, posi1, ...]_0, [posi0, posi1, ...]_1, ..., [posi0, posi1, ...]]_M
    @param calculatedvelocities the calculated velocities for the curve. [[velo0, velo1, ...]_0, [velo0, velo1, ...]_1, ..., [velo0, velo1, ...]]_M
 */
void MotionCurves::calculate(double starttime, const vector<vector<double> >& times, const vector<float>& startpositions, const vector<vector<float> >& positions, float smoothness, int cycletime, vector<vector<double> >& calculatedtimes, vector<vector<float> >& calculatedpositions, vector<vector<float> >& calculatedvelocities)
{
    size_t numjoints = times.size();
    if (numjoints == 0)
        return;
    else if (startpositions.size() < numjoints || positions.size() < numjoints)
    {
        errorlog << "MotionCurves::calculate() failed because times.size(): " << times.size() << " positions.size(): " << positions.size() << " startpositions.size(): " << startpositions.size() << endl;
        return;
    }

    calculatedtimes = vector<vector<double> >();
    calculatedtimes.reserve(numjoints);
    calculatedpositions = vector<vector<float> >();
    calculatedpositions.reserve(numjoints);
    calculatedvelocities = vector<vector<float> >();
    calculatedvelocities.reserve(numjoints);
    for (size_t i=0; i<numjoints; i++)
    {
        vector<double> temptimes;
        vector<float> temppositions;
        vector<float> tempvelocities;
        calculate(starttime, times[i], startpositions[i], positions[i], smoothness, cycletime, temptimes, temppositions, tempvelocities);
        calculatedtimes.push_back(temptimes);
        calculatedpositions.push_back(temppositions);
        calculatedvelocities.push_back(tempvelocities);
    }
}

/*! @brief Calculates a smooth motion curve for several joints with gains. Each joint has its own time vector
 
 The data needs to be of the following format:
 [[time0, time1, ..., timeI]_0, [time0, time1, ..., timeJ]_1, ... , [time0, time1, ..., posiZ]_N]
 [[posi0, posi1, ..., posiI]_0, [posi0, posi1, ..., posiJ]_1, ... , [posi0, posi1, ..., posiZ]_N]
 [[gain0, gain1, ..., gainI]_0, [gain0, gain1, ..., gainJ]_1, ... , [gain0, gain1, ..., gainZ]_N]
 where each time-position-gain tuple has the same length, and there are N joints
 
 @param starttime the time in ms to start moving
 @param times the times in ms to reach the given positions
 @param startpositions the start postion for each joint [start0, start1, ... , startM]
 @param positions the target positions for the curve
 @param gains the target gains for the curve
 @param smoothness a fraction indicating the smoothness of the motion: 0 means linear motion curve, 1 minimises the acceleration and jerk
 @param cycletime the motion cycle time in ms. This is used to decide how many points to generate
 @param calculatedtimes the calculated times for the curve. Do not assume the times will be evenly spaced! [[time0, time1, ...]_0, [time0, time1, ...]_1, ..., [time0, time1, ...]]_M
 @param calculatedpositions the calculated positions for the curve. [[posi0, posi1, ...]_0, [posi0, posi1, ...]_1, ..., [posi0, posi1, ...]]_M
 @param calculatedvelocities the calculated velocities for the curve. [[velo0, velo1, ...]_0, [velo0, velo1, ...]_1, ..., [velo0, velo1, ...]]_M
 @param calculatedgains the calculated gains for the curve [[gain0, gain1, ...]_0, [gain0, gain1, ...]_1, ..., [gain0, gain1, ...]]_M
 */
void MotionCurves::calculate(double starttime, const vector<vector<double> >& times, const vector<float>& startpositions, const vector<vector<float> >& positions, const vector<vector<float> >& gains, float smoothness, int cycletime, vector<vector<double> >& calculatedtimes, vector<vector<float> >& calculatedpositions, vector<vector<float> >& calculatedvelocities, vector<vector<float> >& calculatedgains)
{
    size_t numjoints = times.size();
    if (numjoints == 0)
        return;
    if (startpositions.size() < numjoints || positions.size() < numjoints)
    {
        errorlog << "MotionCurves::calculate() failed because times.size(): " << times.size() << " positions.size(): " << positions.size() << " startpositions.size(): " << startpositions.size() << endl;
        return;
    }
    
    calculatedtimes = vector<vector<double> >();
    calculatedtimes.reserve(numjoints);
    calculatedpositions = vector<vector<float> >();
    calculatedpositions.reserve(numjoints);
    calculatedvelocities = vector<vector<float> >();
    calculatedvelocities.reserve(numjoints);
    calculatedgains = vector<vector<float> >();
    calculatedgains.reserve(numjoints);
    for (size_t i=0; i<numjoints; i++)
    {
        vector<double> temptimes;
        vector<float> temppositions;
        vector<float> tempvelocities;
        vector<float> tempgains;
        calculate(starttime, times[i], startpositions[i], positions[i], gains[i], smoothness, cycletime, temptimes, temppositions, tempvelocities, tempgains);
        calculatedtimes.push_back(temptimes);
        calculatedpositions.push_back(temppositions);
        calculatedvelocities.push_back(tempvelocities);
        calculatedgains.push_back(tempgains);
    }
}

/*! @brief Calculates a smooth trapezoidal curve for a single position
    @param starttime the time in ms to start moving to the given position
    @param stoptime the time in ms to reach the given position
    @param startposition the start postion for the curve
    @param stopposition the stop position for the curve
    @param startvelocity the initial velocity
    @param stopvelocity the final velocity 
    @param smoothness a fraction indicating the smoothness of the motion: 0 means linear motion curve, 1 minimises the acceleration and jerk
    @param cycletime the motion cycle time in ms. This is used to decide how many points to generate
    @param calculatedtimes the calculated times for the curve. Do not assume the times will be evenly spaced!
    @param calculatedpositions the calculated positions for the curve.
 
 Acceleration Profile:
 As--  ---
      |   |    
      |   |    
      |   |     
  ----|   |------|    |----
                 |    |
                 |    |
                  ----  --Af
      |   |      |    |
      t0  t1     t2   tf
 where t1 and t2 move closer to 0.5*tf as the smoothness is increased to 1.
 */
void MotionCurves::calculateTrapezoidalCurve(double starttime, double stoptime, float startposition, float stopposition, float startvelocity, float stopvelocity, float smoothness, int cycletime, vector<double>& calculatedtimes, vector<float>& calculatedpositions, vector<float>& calculatedvelocities)
{
    if (smoothness < 0)
        smoothness = - smoothness;
    if (smoothness > 1)
        smoothness = 1;
    
    double t0 = starttime;
    double t1 = starttime + 0.5*smoothness*(stoptime - starttime);
    double t2 = starttime + (stoptime - starttime)*(1 - 0.5*smoothness);
    double tf = stoptime;
    
    float g0 = startposition;
    float gf = stopposition;
    float v0 = startvelocity;
    float vf = stopvelocity;
    
    // if the time is short or the movement is small or the smoothness is low, don't bother calculating a curve
    if (fabs(tf - t0) < 4*cycletime || fabs(g0 - gf) < 0.05 || smoothness < 0.05)
    {
        calculatedtimes = vector<double> (1, tf);
        if (fabs(tf - t0) > 0.01)
            calculatedvelocities = vector<float> (1, (gf-g0)/(tf-t0));
        else
            calculatedvelocities = vector<float> (1, (gf-g0)/0.01);
        calculatedpositions = vector<float> (1, gf);
        return;
    }
    
    // Calculate the required acceleration magnitudes
    float Af = 2*(gf - g0 - vf*tf + v0*t0 + 0.5*(t1 + t0)*(vf - v0))/(t2*t2 - tf*tf - (t1+t0)*(t2 - tf));
    float As = (vf - v0 - Af*tf + Af*t2)/(t1-t0);
    
    // Calculate the times to calculate the curve points at
    vector<double> times;
    times.reserve(4096);
    for (float t = t0; t <= t1; t += cycletime)
        times.push_back(t);
    for (float t = t2; t < tf; t += cycletime)
        times.push_back(t);
    times.push_back(tf);
    
    // Now calculate the curve itself
    vector<float> positions;
    positions.reserve(times.size());
    vector<float> velocities;
    velocities.reserve(times.size());
    for (unsigned int i=0; i<times.size(); i++)
    {
        float t = times[i];
        if (t <= t1)
        {
            velocities.push_back(As*(t - t0) + v0);
            positions.push_back(0.5*As*t*t - As*t0*t + v0*t + 0.5*As*t0*t0 + g0 - v0*t0);
        }
        else if (t <= t2)
        {
            velocities.push_back(As*(t1 - t0) + v0);
            positions.push_back(As*(t1 - t0)*t + v0*t - 0.5*As*t1*t1 + 0.5*As*t0*t0 + g0 - v0*t0);
        }
        else
        {
            velocities.push_back(Af*t + As*(t1 - t0) - Af*t2 + v0);
            positions.push_back(0.5*Af*t*t + As*(t1 - t0)*t - Af*t2*t + v0*t + 0.5*Af*t2*t2 - 0.5*As*t1*t1 + 0.5*As*t0*t0 + g0 - v0*t0);
        }
    }

    // Finally copy the result to the output vectors
    calculatedtimes = times;
    calculatedpositions = positions;
    calculatedvelocities = velocities;
}
                                  
float MotionCurves::calculateFinalVelocity(float starttime, float stoptime, float nextstoptime, float startposition, float stopposition, float nextstopposition)
{
    return (calculateAvgVelocity(starttime, stoptime, startposition, stopposition) + calculateAvgVelocity(stoptime, nextstoptime, stopposition, nextstopposition))/2.0;
}

/*! @brief Calculates the average velocity in moving from startposition to stopposition over starttime to stoptime
    @param startposition
    @param stopposition
    @param starttime the start time in ms
    @param stoptime the stop time in ms
 */
float MotionCurves::calculateAvgVelocity(float starttime, float stoptime, float startposition, float stopposition)
{
    if (fabs(starttime - stoptime) > 0.01)
        return (stopposition - startposition)/(stoptime - starttime);
    else
        return (stopposition - startposition)/0.01;
}


