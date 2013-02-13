/*! @file HeadLogic.h
    @brief Logic for head behaviours.
    Main methods involve porting of a vector of values such as: times since objects last seen, object locations, object head movement costs.
    The indexing for each of these lists is identical. The method getObject(int index) gets the object corresponding to the values in the
    slot [index] in each list.

    @author Jake Fountain

 Copyright (c) 2012

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


#ifndef HEADLOGIC_H
#define HEADLOGIC_H


#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/FieldObjects/Object.h"
#include "NUPlatform/NUCamera/NUCameraData.h"

#include "Infrastructure/NUSensorsData/NUSensorsData.h"


#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"


#include "Tools/Math/General.h"


#include <vector>
//class Object;
//class NUCameraData;


class HeadLogic{

public:

    static HeadLogic* getInstance();

    enum ObjectTypeID{
        STATIONARY_OBJECT = 0,
        MOBILE_OBJECT = 1,
        AMBIGUOUS_OBJECT = 2
    };



    HeadLogic();
    ~HeadLogic();

    
    Object* getObject(int object_type, int object_enum_value);    
    Object* getObject(int index);    
    int getObjectType(int index);
    std::vector<float> getObjectLocation(int object_type,int object_enum_value);
    std::vector<float> calculateStationaryObjectLocation(StationaryObject ob);
    std::vector<float> calculateMobileObjectLocation(MobileObject ob);
    std::vector<float> calculateAmbiguousObjectLocation(AmbiguousObject ob);
    float getTimeSinceObjectLastSeen(int object_type, int object_enum_type);
    std::vector<float> getSelfLocation();    

    std::vector<float> getCostList(float scale_x,float scale_y);

    Object* cheapestObject(float scale_x, float scale_y);
    Object* cheapestStationaryObject(float scale_x, float scale_y);
    Object* cheapestMobileObject(float scale_x, float scale_y);
    Object* cheapestAmbiguousObject(float scale_x, float scale_y);
    float getRequiredHeadMovementAngle(std::vector<float> self_location, int ob_type, int object_num, float scale_x, float scale_y);

//Some vector operators
    float cartNorm(float x, float y, float z=0);
    float innerProd(std::vector<float> x1, std::vector<float> x2);

    std::vector<float> getTimeSinceLastSeenSummary();
    std::vector<float> getSimpleObLocSummary();

    std::vector<std::vector<float> > getObLocSummary();

    //Vector of all relevant object vectors, indexed via the enum types.
    std::vector<std::vector<int> > relevantObjects;


    std::vector<float> calculateStationaryPolarObjectLocation(StationaryObject* ob);
    std::vector<float> calculateMobilePolarObjectLocation(MobileObject* ob);
    std::vector<float> calculateAmbiguousPolarObjectLocation(AmbiguousObject* ob);
    std::vector<float> getSimplePolarObLocSummary();
    std::vector<std::vector<float> > getPolarObLocSummary();

    std::vector<int> getValidObjectsToLookAt();
private:




    //Camera field of view constants
    float m_CAMERA_FOV_X;//rad
    float m_CAMERA_FOV_Y;
};

#endif // HEADLOGIC_H
