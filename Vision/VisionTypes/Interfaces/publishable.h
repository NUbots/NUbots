#ifndef PUBLISHABLE_H
#define PUBLISHABLE_H

#include "Infrastructure/FieldObjects/FieldObjects.h"

class Publishable {

protected:
    /*!
      @brief pushes the object to the external field objects.
      @param fieldobjects a pointer to the global list of field objects.
      @param timestamp the image timestamp.
      @return the success of the operation.
      */
    virtual bool addToExternalFieldObjects(FieldObjects* fieldobjects, float timestamp) const = 0;
};

#endif // PUBLISHABLE_H
