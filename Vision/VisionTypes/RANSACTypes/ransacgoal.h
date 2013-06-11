/*! @file KickJob.h
    @brief Declaration of KickJob class.
 
    @class KickJob
    @brief A base class to encapsulate jobs issued for the kick module.
 
    @author Jason Kulk
 
  Copyright (c) 2009 Jason Kulk
 
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

#ifndef KICKJOB_H
#define KICKJOB_H

#include "../MotionJob.h"
#include <vector>


class KickJob : public MotionJob
{
public:
    KickJob(double time, const std::vector<float>& kickposition, const std::vector<float>& kicktarget);
    KickJob(double time, std::istream& input);
    ~KickJob();
    
    void setKick(double time, const std::vector<float>& kickposition, const std::vector<float>& kicktarget);
    void setKickPosition(double time, const std::vector<float>& kickposition);
    void setKickTarget(const std::vector<float>& kicktarget);
    void getKick(double& time, std::vector<float>& kickposition, std::vector<float>& kicktarget);
    void getKickPosition(double& time, std::vector<float>& kickposition);
    void getKickTarget(std::vector<float>& kicktarget);
    
    virtual void summaryTo(std::ostream& output);
    virtual void csvTo(std::ostream& output);
    
    friend std::ostream& operator<<(std::ostream& output, const KickJob& job);
    friend std::ostream& operator<<(std::ostream& output, const KickJob* job);
protected:
    virtual void toStream(std::ostream& output) const;
private:
    std::vector<float> m_kick_position;                 //!< the kick position [x(cm), y(cm)]
    std::vector<float> m_kick_target;                   //!< the kick target relative from the *current* position [x(cm) y(cm)]
};

#endif

