/*! @file Jobs.h
    @brief A convience header that will include the Job interface.
 
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

#ifndef JOBS_H
#define JOBS_H

#include "Jobs/JobList.h"
#include "Jobs/Job.h"

#include "Jobs/CameraJob.h"
#include "Jobs/CameraJobs/ChangeCameraSettingsJob.h"

#include "Jobs/VisionJob.h"
#include "Jobs/VisionJobs/SaveImagesJob.h"

#include "Jobs/LocalisationJob.h"
#include "Jobs/BehaviourJob.h"
#include "Jobs/MotionJob.h"
#include "Jobs/MotionJobs/BlockJob.h"
#include "Jobs/MotionJobs/KickJob.h"
#include "Jobs/MotionJobs/HeadJob.h"
#include "Jobs/MotionJobs/HeadNodJob.h"
#include "Jobs/MotionJobs/HeadPanJob.h"
#include "Jobs/MotionJobs/SaveJob.h"
#include "Jobs/MotionJobs/WalkJob.h"
#include "Jobs/MotionJobs/WalkToPointJob.h"
#include "Jobs/MotionJobs/WalkParametersJob.h"

#include "Jobs/LightJob.h"
#include "Jobs/LightJobs/ChestLedJob.h"
#include "Jobs/LightJobs/LEarLedJob.h"
#include "Jobs/LightJobs/LEyeLedJob.h"
#include "Jobs/LightJobs/LFootLedJob.h"
#include "Jobs/LightJobs/REarLedJob.h"
#include "Jobs/LightJobs/REyeLedJob.h"
#include "Jobs/LightJobs/RFootLedJob.h"

#include "Jobs/SoundJob.h"
#include "Jobs/SystemJob.h"
#include "Jobs/OtherJob.h"


#endif

