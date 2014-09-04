/*! @file WalkParameterJob.h
    @brief Declaration of WalkParameterJob class.
 
    @class WalkParameterJob
    @brief A base class to encapsulate jobs issued to change parameters inside the walk module.
 
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

#ifndef WALKPARAMETERJOB_H
#define WALKPARAMETERJOB_H

#include "../MotionJob.h"
#include "Motion/Walks/WalkParameters.h"

#include <vector>


class WalkParametersJob : public MotionJob
{
public:
    WalkParametersJob(const WalkParameters& walkparameters);
    WalkParametersJob(std::istream& input);
    ~WalkParametersJob();
    
    void setWalkParameters(const WalkParameters& walkparameters);
    void getWalkParameters(WalkParameters& walkparameters);
    
    virtual void summaryTo(std::ostream& output);
    virtual void csvTo(std::ostream& output);
    
    friend std::ostream& operator<<(std::ostream& output, const WalkParametersJob& job);
    friend std::ostream& operator<<(std::ostream& output, const WalkParametersJob* job);
protected:
    virtual void toStream(std::ostream& output) const;
private:
    WalkParameters m_walk_parameters;               //!< the walk parameters to give to the walk engine
};

#endif

