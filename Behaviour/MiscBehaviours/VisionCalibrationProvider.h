/*! @file VisionCalibrationProvider.h
    @brief Declaration of a behaviour provider for vision calibration purposes
 
    @class VisionCalibrationProvider
    @brief A special behaviour for calibrating images.
 
    The chest button is used to start and stop saving images. 
    A short click saves images while rotating exposure, and a long one uses the current settings while saving.
    The left and right buttons scroll through the various motions to perform while saving images.

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

#ifndef VISIONCALIBRATIONPROVIDER_H
#define VISIONCALIBRATIONPROVIDER_H

#include "../BehaviourProvider.h"

class JobList;
class NUSensorsData;
class NUActionatorsData;
class FieldObjects;
class GameInformation;
class TeamInformation;

#include <vector>
#include <string>

class VisionCalibrationProvider : public BehaviourProvider
{
public:
    VisionCalibrationProvider(Behaviour* manager);
    ~VisionCalibrationProvider();
protected:
    void doBehaviour();
    void doSelectedMotion();
private:
    int m_selection_index;
    int m_num_motions;
    bool m_saving_images;
    
    BehaviourProvider* m_chase_ball;
};


#endif

