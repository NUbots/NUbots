/* 
 * File:   VisionObject.h
 * Author: Andrew
 *
 * Created on 18 December 2013, 2:55 PM
 * replaces the previous FieldObject.h class
 * this currently a dummy class to be integrated with what the vision guys are working on :)
 */

#ifndef VISIONOBJECT_H
#define	VISIONOBJECT_H

class VisionObjects {
public:
    VisionObjects() {} //constructor
    
    unsigned int getID() const { //originally AmbiguousObject.getID
        return 0;
    }

    enum StationaryFieldObjectID {
        NUM_STAT_FIELD_OBJECTS = 27
    };
    
    enum AmbiguousObjectID {
         NUM_AMBIGUOUS_FIELD_OBJECTS = 11 //reference an enum variable the same way you'd reference a static const int (as follows).. VisionObject::NUM_AMBIGUOUS_FIELD_OBJECTS
    };
    
};

#endif	/* VISIONOBJECT_H */

