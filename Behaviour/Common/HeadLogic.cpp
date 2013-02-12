/*! @file HeadLogic.cpp
    @b
 maths and logic for head behaviours.

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


#include "HeadLogic.h"

#include <cmath>
#include <vector>
#include <string>



HeadLogic* HeadLogic::getInstance(){
    static HeadLogic* instance = new HeadLogic();
    return instance;
}

HeadLogic::HeadLogic(){
    //cout<<"Head logic constructor start"<<endl;
    NUCameraData cameraSpecs(string(/*CONFIG_DIR*/ "Config/Darwin") + "CameraSpecs.cfg");
    m_CAMERA_FOV_X = cameraSpecs.m_horizontalFov;
    m_CAMERA_FOV_Y = cameraSpecs.m_verticalFov;

    //Relevant object lists:
        // teammates; ball; goals; nearest ambiguous ob.

    std::vector<int> relevantSOb;
    std::vector<int> relevantMOb;
    std::vector<int> relevantAOb;

    relevantSOb.push_back(FieldObjects::FO_BLUE_LEFT_GOALPOST);
    relevantSOb.push_back(FieldObjects::FO_BLUE_RIGHT_GOALPOST);
    relevantSOb.push_back(FieldObjects::FO_YELLOW_LEFT_GOALPOST);
    relevantSOb.push_back(FieldObjects::FO_YELLOW_RIGHT_GOALPOST);
    /*relevantSOb.push_back(FieldObjects::FO_BLUE_BEACON);
    relevantSOb.push_back(FieldObjects::FO_YELLOW_BEACON);*/

	relevantMOb.push_back(FieldObjects::FO_BALL);
    /*relevantMOb.push_back(FieldObjects::FO_BLUE_ROBOT_1);
	relevantMOb.push_back(FieldObjects::FO_BLUE_ROBOT_2);
	relevantMOb.push_back(FieldObjects::FO_BLUE_ROBOT_3);
	relevantMOb.push_back(FieldObjects::FO_BLUE_ROBOT_4);
	relevantMOb.push_back(FieldObjects::FO_PINK_ROBOT_1);
    relevantMOb.push_back(FieldObjects::FO_PINK_ROBOT_2);
    relevantMOb.push_back(FieldObjects::FO_PINK_ROBOT_3);
    relevantMOb.push_back(FieldObjects::FO_PINK_ROBOT_4);*/

    //relevantAOb.push_back(0);
    //relevantAOb.push_back(1);
    //relevantAOb.push_back(2);
    //relevantAOb.push_back(3);


	/*Old attempt at listing relevant objects.
    int relevantStationaryObjects[] = {FieldObjects::F0_BLUE_LEFT_GOALPOST,
                                          FieldObjects::F0_BLUE_RIGHT_GOALPOST     ,
                                          FieldObjects::F0_YELLOW_LEFT_GOALPOST    ,
                                          FieldObjects::F0_YELLOW_RIGHT_GOALPOST   ,
                                          FieldObjects::F0_BLUE_BEACON             ,
                                          FieldObjects::F0_YELLOW_BEACON};
    int relevantMobileObjects[] = {FieldObjects::FO_BALL                ,
                                      FieldObjects::FO_BLUE_ROBOT_1 		,
                                      FieldObjects::FO_BLUE_ROBOT_2 		,
                                      FieldObjects::FO_BLUE_ROBOT_3		,
                                      FieldObjects::FO_BLUE_ROBOT_4        ,
                                      FieldObjects::FO_PINK_ROBOT_1 		,
                                      FieldObjects::FO_PINK_ROBOT_2 		,
                                      FieldObjects::FO_PINK_ROBOT_3 		,
                                      FieldObjects::FO_PINK_ROBOT_4 };
    int relevantAmbiguousObjects[] = {0,1,2,3};

    //Create vectors corresponding to lists of relevant objects
    vector<int> relevantSOb(relevantStationaryObjects,relevantStationaryObjects+sizeof(relevantStationaryObjects)/sizeof(int));
    vector<int> relevantMOb(relevantMobileObjects,relevantMobileObjects+sizeof(relevantMobileObjects)/sizeof(int));
    vector<int> relevantAOb(relevantAmbiguousObjects,relevantAmbiguousObjects+sizeof(relevantAmbiguousObjects)/sizeof(int));*/

    relevantObjects.push_back(relevantSOb);
    relevantObjects.push_back(relevantMOb);
    relevantObjects.push_back(relevantAOb);

}
HeadLogic::~HeadLogic(){}

/*! @brief gets object using internal ID system
    */
Object* HeadLogic::getObject(int object_type, int object_enum_value){
    switch (object_type){
        case 0:
            return &(Blackboard->Objects->stationaryFieldObjects[object_enum_value]);
        case 1:
            return &(Blackboard->Objects->mobileFieldObjects[object_enum_value]);
        case 2:
            return &(Blackboard->Objects->ambiguousFieldObjects[object_enum_value]);
    }
}
/*! @brief
     * Gets the type of the ith object from the list of interesting objects. Self not gettable.
     * */
int HeadLogic::getObjectType(int index){

    std::vector<std::vector<int> > objects_of_interest;
    std::vector<int> object_marker(2,0);
        //Interesting objects:
    for (int object_type = 0; object_type<3;object_type++){
        //For each object type go through the list of interesting objects and add times to result vector.
        for (int i = 0; i < relevantObjects[object_type].size();i++){
            object_marker[0] = object_type;
            object_marker[1] = relevantObjects[object_type][i];
            objects_of_interest.push_back(object_marker);
        }
    }

    return objects_of_interest[index][0];



}

/*! @brief
     * Gets the ith object from the list of interesting objects. Self not gettable.
     * */
Object* HeadLogic::getObject(int index){
    std::vector<std::vector<int> > objects_of_interest;
    std::vector<int> object_marker(2,0);
	    //Interesting objects:
	for (int object_type = 0; object_type<3;object_type++){
	    //For each object type go through the list of interesting objects and add times to result vector.
	    for (int i = 0; i < relevantObjects[object_type].size();i++){
	    	object_marker[0] = object_type;
	    	object_marker[1] = relevantObjects[object_type][i];
	    	objects_of_interest.push_back(object_marker);
	    }
	}
    return getObject(objects_of_interest[index][0],objects_of_interest[index][1]);

}


/*! @brief Returns a vector (x,y) of the relative location of a stationary object. Self not localised here.
  Eg. getObjectLocation(HeadLogic::STATIONARY_OBJECT,FieldObjects::FO_BLUE_LEFT_GOALPOST);
*/
std::vector<float> HeadLogic::getObjectLocation(int object_type,int object_enum_value){
    switch (object_type){
        case 0:
            return calculateStationaryObjectLocation((StationaryObject)Blackboard->Objects->stationaryFieldObjects[object_enum_value]);
        case 1:
            return calculateMobileObjectLocation((MobileObject)Blackboard->Objects->mobileFieldObjects[object_enum_value]);
        case 2:
            return calculateAmbiguousObjectLocation((AmbiguousObject)Blackboard->Objects->ambiguousFieldObjects[object_enum_value]);
    }

}

/*! @brief Returns a vector (x,y) of the relative location of a field object.
*/

std::vector<float> HeadLogic::calculateStationaryObjectLocation(StationaryObject ob){
    //Initialise location vector:
    std::vector<float> loc(2,0);
    //init polar coords:
    float r,theta;
    //Get polar location
    vector<float> polarLoc = calculateStationaryPolarObjectLocation(&ob);


    r= polarLoc[0];
    theta = polarLoc[1];


    //polar->cartesian
    loc[0] = r*cos(theta);
    loc[1] = r*sin(theta);
    return loc;
}
std::vector<float> HeadLogic::calculateMobileObjectLocation(MobileObject ob){
    //Initialise location vector:
    std::vector<float> loc(2,0);
    //init polar coords:
    float r,theta;
    //Get polar location
    vector<float> polarLoc = calculateMobilePolarObjectLocation(&ob);


    r= polarLoc[0];
    theta = polarLoc[1];


    //polar->cartesian
    loc[0] = r*cos(theta);
    loc[1] = r*sin(theta);
    return loc;
}

std::vector<float> HeadLogic::calculateAmbiguousObjectLocation(AmbiguousObject ob){
    //Initialise location vector:
    std::vector<float> loc(2,0);
    //init polar coords:
    float r,theta;
    //Get polar location
    vector<float> polarLoc = calculateAmbiguousPolarObjectLocation(&ob);


    r= polarLoc[0];
    theta = polarLoc[1];


    //polar->cartesian
    loc[0] = r*cos(theta);
    loc[1] = r*sin(theta);
    return loc;
}


/*! @brief Returns a vector (r,theta) of the relative location of a field object.
*/

std::vector<float> HeadLogic::calculateStationaryPolarObjectLocation(StationaryObject* ob){
    Self* self =&(Blackboard->Objects->self);
    float bearing = self->CalculateBearingToStationaryObject(*ob);
    float distance = self->CalculateDistanceToStationaryObject(*ob);
    vector<float> polar;
    polar.push_back(distance);
    polar.push_back(bearing);
    //cout<< "HeadLogic::calculateStationaryPolarObjectLocation : Location (r,theta) = ["<<polar[0]<<", "<<polar[1]<<"]"<<endl;
    return polar;
}
std::vector<float> HeadLogic::calculateMobilePolarObjectLocation(MobileObject* ob){
    std::vector<float> loc;
    if(!ob->isObjectVisible()){
        //Get location from vision
        loc.push_back(ob->measuredDistance());
        loc.push_back(ob->measuredBearing());
        //cout<< "HeadLogic::calculateMobilePolarObjectLocation : Measured Location (r,theta) = ["<<loc[0]<<", "<<loc[1]<<"]"<<endl;
    } else{
        //Estimate location
        loc.push_back(ob->estimatedDistance());
        loc.push_back(ob->estimatedBearing());
        //cout<< "HeadLogic::calculateMobilePolarObjectLocation : Estimated Location (r,theta) = ["<<loc[0]<<", "<<loc[1]<<"]"<<endl;
    }

    return loc;
}
std::vector<float> HeadLogic::calculateAmbiguousPolarObjectLocation(AmbiguousObject* ob){
    std::vector<float> loc;
    if(!ob->isObjectVisible()){
        //Get location from vision
        loc.push_back(ob->measuredDistance());
        loc.push_back(ob->measuredBearing());
        //cout<< "HeadLogic::calculateAmbiguousPolarObjectLocation : Measured Location = ["<<loc[0]<<", "<<loc[1]<<"]"<<endl;
    } else{
        //Estimate location
        loc.push_back(ob->estimatedDistance());
        loc.push_back(ob->estimatedBearing());
        //cout<< "HeadLogic::calculateAmbiguousPolarObjectLocation : Estimated Location = ["<<loc[0]<<", "<<loc[1]<<"]"<<endl;
    }

    return loc;
}

/*! @brief Returns a simple vector of relevant object locations in the form [self.heading,self.locx,self.locy, r1,theta1,r2,theta2,...,rn,thetan].
  All locations are relative except self location.
*/
std::vector<float> HeadLogic::getSimplePolarObLocSummary(){
    //Get standard summary:
    std::vector<std::vector<float> > summary = getPolarObLocSummary();

    //Deconstruct vector elements of the standard summary into simple format:
    std::vector<float> result;
    result.push_back(Blackboard->Objects->self.Heading());
    for (int i = 0; i < summary.size();i++){
        result.push_back(summary[i][0]);
        result.push_back(summary[i][1]);
    }
    return result;
}

/*! @brief Returns a vector of relevant object locations in the form [[r1,theta1],[r2,theta2],...,[rn,thetan]].
  All locations are relative except self location.
*/

std::vector<std::vector<float> > HeadLogic::getPolarObLocSummary(){
    std::vector<std::vector<float> > result;

    result.push_back(getSelfLocation());

    int object_type = 0;
    for (int i = 0; i < relevantObjects[object_type].size();i++){
            result.push_back(calculateStationaryPolarObjectLocation((StationaryObject*)getObject(object_type,relevantObjects[object_type][i])));
    }
    object_type++;
    for (int i = 0; i < relevantObjects[object_type].size();i++){
            result.push_back(calculateMobilePolarObjectLocation((MobileObject*)getObject(object_type,relevantObjects[object_type][i])));
    }
    object_type++;
    for (int i = 0; i < relevantObjects[object_type].size();i++){
            result.push_back(calculateAmbiguousPolarObjectLocation((AmbiguousObject*)getObject(object_type,relevantObjects[object_type][i])));
    }

    return result;
}




/*! @brief Returns the time since last seen for object of type object_type (stat,mobile or ambig).
*/
float HeadLogic::getTimeSinceObjectLastSeen(int object_type, int object_enum_value){
    switch (object_type){
        case 0:
            return Blackboard->Objects->stationaryFieldObjects[object_enum_value].TimeSinceLastSeen();
        case 1:
            return Blackboard->Objects->mobileFieldObjects[object_enum_value].TimeSinceLastSeen();
        case 2:
            return Blackboard->Objects->ambiguousFieldObjects[object_enum_value].TimeSinceLastSeen();
    }


}

/*! @brief Get absolute location of self.
*/
std::vector<float> HeadLogic::getSelfLocation(){
    Self* self =&(Blackboard->Objects->self);
    std::vector<float> loc(2,0);

    loc[0] = self->wmX();
    loc[1] = self->wmY();

    return loc;

}

/*! @brief Gets a list of costs to look at each object of interest.
    @param scale_x is the horizontal size of the box which the object must be placed in on the vision field
    before it is counted as "seen".
           scale_y is the vertical size of the box.
            Both parameters are specified as proportions of the vision field size.
*/
std::vector<float> HeadLogic::getCostList(float scale_x,float scale_y){
    std::vector<float> self_location = getSelfLocation();
    std::vector<float> costs;
    for (int object_type = 0; object_type<3; object_type++){
        for (int i = 0; i < relevantObjects[object_type].size();i++){
            float next_cost = getRequiredHeadMovementAngle(self_location,object_type,relevantObjects[object_type][i],scale_x,scale_y);
            costs.push_back(next_cost);
        }
    }
    return costs;
}

/*! @brief Get the object with cheapest head movement to place the object inside the box of size
    (scale_x*vision_width) by (scale_y*vision_height), centred on the centre of the vision field.
 If scale_x = scale_y = 0 then returns the cheapest object to centre vision on.
 Returns vector of ints: [cheapest_object_type, cheapest_object].
*/
Object* HeadLogic::cheapestObject(float scale_x, float scale_y){
    int cheapest_object;
    float cheapest_cost;
    int cheapest_object_type;
    std::vector<float> self_location = getSelfLocation();

    for (int object_type = 0; object_type<3; object_type++){
        for (int i = 0; i < relevantObjects[object_type].size();i++){
            float next_cost = getRequiredHeadMovementAngle(self_location,object_type,relevantObjects[object_type][i],scale_x,scale_y);
            if ( next_cost!=0 && next_cost < cheapest_cost ){
                cheapest_object_type = object_type;
                cheapest_object = i;
                cheapest_cost = next_cost;
            }
        }
    }
    Object* result = getObject(cheapest_object_type,cheapest_object);
    return result;
}

Object* HeadLogic::cheapestStationaryObject(float scale_x, float scale_y){
    int cheapest_object;
    float cheapest_cost;
    int object_type = STATIONARY_OBJECT;
    std::vector<float> self_location = getSelfLocation();


   for (int i = 0; i < relevantObjects[object_type].size();i++){
        float next_cost = getRequiredHeadMovementAngle(self_location,object_type,relevantObjects[object_type][i],scale_x,scale_y);

        if ( next_cost!=0 && next_cost < cheapest_cost ){
            cheapest_object = i;
            cheapest_cost = next_cost;
        }

    }
    Object* result = getObject(object_type,cheapest_object);
    return result;
}

Object* HeadLogic::cheapestMobileObject(float scale_x, float scale_y){
    int cheapest_object;
    float cheapest_cost;
    int object_type = MOBILE_OBJECT;
    std::vector<float> self_location = getSelfLocation();


    for (int i = 0; i < relevantObjects[object_type].size();i++){
        float next_cost = getRequiredHeadMovementAngle(self_location,object_type,relevantObjects[object_type][i],scale_x,scale_y);

        if ( next_cost!=0 && next_cost < cheapest_cost ){
            cheapest_object = i;
            cheapest_cost = next_cost;
        }

    }
    Object* result = getObject(object_type,cheapest_object);
    return result;
}
Object* HeadLogic::cheapestAmbiguousObject(float scale_x, float scale_y){
    int cheapest_object;
    float cheapest_cost;
    int object_type = AMBIGUOUS_OBJECT;
    std::vector<float> self_location = getSelfLocation();


    for (int i = 0; i < relevantObjects[object_type].size();i++){
        float next_cost = getRequiredHeadMovementAngle(self_location,object_type,relevantObjects[object_type][i],scale_x,scale_y);

        if ( next_cost!=0 && next_cost < cheapest_cost ){
            cheapest_object = i;
            cheapest_cost = next_cost;
        }

    }
    Object* result = getObject(object_type,cheapest_object);
    return result;
}




/*! @brief Calculates the required head movement to move the object's image within the box on screen of size
    (scale_x*vision_width) by (scale_y*vision_height).
*/
float HeadLogic::getRequiredHeadMovementAngle(std::vector<float> self_location, int ob_type, int object_num, float scale_x, float scale_y){
    //get bow width and height
    float box_width = m_CAMERA_FOV_X*scale_x;//rad
    float box_height = m_CAMERA_FOV_Y*scale_y;
    //get absolute object location:
    std::vector<float> object_field_loc = getObjectLocation(ob_type,object_num);
    std::vector<float> object_relative_loc;
    //x coord:
    object_relative_loc.push_back(object_field_loc[0]-self_location[0]);
    //y coord:
    object_relative_loc.push_back(object_field_loc[1]-self_location[1]);
    //Get camera height:
    float cameraHeight;
    Blackboard->Sensors->getCameraHeight(cameraHeight);
    //z coordinate of object from camera.
    object_relative_loc.push_back(-cameraHeight);
    //Object pitch and yaw:
    float object_yaw = atan2(object_relative_loc[1],object_relative_loc[0]);
    float object_pitch = atan2(object_relative_loc[2],object_relative_loc[0]);


    //Find camera (unit) vector:
    float cameraPitch, cameraYaw;
    Blackboard->Sensors->getPosition(NUSensorsData::HeadPitch,cameraPitch);
    Blackboard->Sensors->getPosition(NUSensorsData::HeadYaw,cameraYaw);

    float result = 0;
    float relativeYaw =object_yaw-cameraYaw;
    float relativePitch = object_pitch-cameraPitch;

    if (fabs(relativeYaw)<box_width/2 && fabs(relativePitch)<box_height/2){
        //if inside box, the result is zero
        return 0;
    }else if(fabs(relativeYaw)<box_width/2){
        //If in vertical column of box
        result = fabs(relativePitch)-box_height/2;

    }else if (fabs(relativePitch)<box_height/2){
        //if in horizontal column of box
        result = fabs(relativeYaw)-box_width/2;
    } else {
        //If diagonal to box, calculate the distances to the four box corners and take the minimum.
        float d = cartNorm(relativeYaw+box_width/2,relativePitch+box_height/2,0);
        result = d;
        d = cartNorm(relativeYaw-box_width/2,relativePitch+box_height/2);
        if (d<result){result = d;}
        d = cartNorm(relativeYaw-box_width/2,relativePitch-box_height/2);
        if (d<result){result = d;}
        d = cartNorm(relativeYaw+box_width/2,relativePitch-box_height/2);
        if (d<result){result = d;}
    }
    if (result<0){
        //Result can be negative if point in column.
        result =0;
    }

    return result;
}
    /*! @brief Some vector operations*/
float HeadLogic::cartNorm(float x, float y, float z){
    return sqrt(x*x+y*y+z*z);
}
float HeadLogic::innerProd(std::vector<float> x1, std::vector<float> x2){
    float ip = 0;
    for (int i = 0; i<x1.size();i++){
        ip+= x1[i]*x2[i];
    }
    return ip;
}


/*! @brief Returns a simple vector of relevant object times since last seen in the form [t1,t2,...,tn] in seconds.
  Object order matches with the objects returned by the location summary methods.
*/
std::vector<float> HeadLogic::getTimeSinceLastSeenSummary(){
     //cout<<"Head logic timesumm start"<<endl;
    std::vector<float> result;
    //Convention for time last seen of self always set to 0. This is to match up indices with the location summary vector.
    //result.push_back(0);
    //Interesting objects:
    for (int object_type = 0; object_type<3;object_type++){
        //For each object type go through the list of interesting objects and add times to result vector.
        for (int i = 0; i < relevantObjects[object_type].size();i++){
            result.push_back(getTimeSinceObjectLastSeen(object_type,relevantObjects[object_type][i])/1000.0/*Convert from msec to sec*/);
        }
    }
    return result;
    //cout<<"Head logic timesumm end"<<endl;
}


/*! @brief Returns a simple vector of relevant object locations in the form [x1,y1,x2,y2,...,xn,yn].
  All locations are relative except self location.
*/
std::vector<float> HeadLogic::getSimpleObLocSummary(){
    //Get standard summary:
    std::vector<std::vector<float> > summary = getObLocSummary();

    //Deconstruct vector elements of the standard summary into simple format:
    std::vector<float> result;
    for (int i = 0; i < summary.size();i++){
        result.push_back(summary[i][0]);
        result.push_back(summary[i][1]);
    }
    return result;
}

/*! @brief Returns a vector of relevant object locations in the form [[x1,y1],[x2,y2],...,[xn,yn]].
  All locations are relative except self location.
*/

std::vector<std::vector<float> > HeadLogic::getObLocSummary(){
    std::vector<std::vector<float> > result;

    result.push_back(getSelfLocation());
    for (int object_type = 0; object_type<3;object_type++){
        for (int i = 0; i < relevantObjects[object_type].size();i++){
            result.push_back(getObjectLocation(object_type,relevantObjects[object_type][i]));
        }
    }
    return result;
}


std::vector<int> HeadLogic::getValidObjectsToLookAt()
{
    std::vector<float> bearings;

    int object_type = 0;
    for (int i = 0; i < relevantObjects[object_type].size();i++){
        bearings.push_back(calculateStationaryPolarObjectLocation((StationaryObject*)getObject(object_type,relevantObjects[object_type][i]))[1]);
    }
    object_type++;
    for (int i = 0; i < relevantObjects[object_type].size();i++){
        bearings.push_back(calculateMobilePolarObjectLocation((MobileObject*)getObject(object_type,relevantObjects[object_type][i]))[1]);
    }
    object_type++;
    for (int i = 0; i < relevantObjects[object_type].size();i++){
        bearings.push_back(calculateAmbiguousPolarObjectLocation((AmbiguousObject*)getObject(object_type,relevantObjects[object_type][i]))[1]);
    }
    vector<int> validities;

    for(int i = 0;i<bearings.size();i++){
        if(bearings[i]*bearings[i] <= 2.46/*(pi/2)^2*/)
            validities.push_back(1);
        else
            validities.push_back(0);
    }

    return validities;

}







