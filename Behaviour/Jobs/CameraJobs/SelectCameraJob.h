/*! @file SelectCameraJob.h
    @brief Declaration of base SelectCameraJob class.
 
    @class SelectCameraJob
    @brief A class to encapsulate jobs issued for the camera switching module.
 
    All camera jobs should inherit from this base class.
 
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

#ifndef CAMERAJOB_H
#define CAMERAJOB_H

#include "Job.h"

class SelectCameraJob : public CameraJob
{
public:
    SelectCameraJob(double time, int camnumber) : CameraJob(CAMERA_SELECT_CAMERA, time), m_camera_number(camnumber){};
    ~SelectCameraJob() {};
    
    void setCameraNumber(double time, int camnumber) {m_job_time = time; m_camera_number = camnumber;};
    void getCameraNumber(double& time, int& camnumber) {time = m_job_time; camnumber = m_camera_number;};
    
    virtual void summaryTo(ostream& output);
    virtual void csvTo(ostream& output);
    
    virtual ostream& operator<< (ostream& output) {return output;};
    virtual istream& operator>> (istream& input) {return input;};
private:
    int m_camera_number;
};

#endif

