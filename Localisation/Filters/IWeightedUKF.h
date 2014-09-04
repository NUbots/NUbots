/*! @file UKF.h
 @brief Declaration of IKalmanFilter interface class

 @class IKalmanFilter
 @brief Interface class for all Kalman Filter classes.

This contains the basic functions that all implementations can and must use.

 @author Steven Nicklin

 Copyright (c) 2012 Steven Nicklin

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

#pragma once
#include "IWeightedKalmanFilter.h"
#include "IUKF.h"

class IWeightedUKF: public IWeightedKalmanFilter, IUKF
{
public:
    virtual ~IWeightedUKF()
    {
    }

    virtual IKalmanFilter* Clone() = 0;

protected:
    IWeightedUKF(IKFModel* model): IKalmanFilter(model), IWeightedKalmanFilter(model), IUKF(model)
    {
    }

    IUKF(const IUKF& source): IKalmanFilter(source), IWeightedKalmanFilter(source), IUKF(source)
    {
    }
};
