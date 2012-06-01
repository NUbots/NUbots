/**
* @file MassCalibration.h
* Declaration of a class for representing the relative positions and masses of mass points.
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
* @note edited by <a href="mailto:shannon.fenn@uon.edu.au">Shannon Fenn</a>
*/

#ifndef MASSCALIBRATION_H
#define MASSCALIBRATION_H

#include "Tools/Math/Vector3.h"
#include "debug.h"
#include "nubotdataconfig.h"

#include <fstream>

using namespace std;

class MassCalibration
{
public:
  enum Limb
  {
      neck,
      head,
      shoulderLeft,
      bicepsLeft,
      elbowLeft,
      foreArmLeft,
      shoulderRight,
      bicepsRight,
      elbowRight,
      foreArmRight,
      pelvisLeft,
      hipLeft,
      thighLeft,
      tibiaLeft,
      ankleLeft,
      footLeft,
      pelvisRight,
      hipRight,
      thighRight,
      tibiaRight,
      ankleRight,
      footRight,
      torso,
      numOfLimbs
  };

  /**
  * Information on the mass distribution of a limb of the robot.
  */
  class MassInfo
  {
  public:
    float mass; /**< The mass of this limb. */
    Vector3<> offset; /**< The offset of the center of mass of this limb relative to its hinge. */

    /**
    * Default constructor.
    */
    MassInfo() : mass(0), offset() {}
    
    //! @brief Stream extraction operator for a mass.
    friend istream& operator>> (istream& input, MassInfo& m)
    {
        char name[25];
        // read in the part name
        input.getline(name, 128, ':');
        // read in the values
        input >> m.mass;
        input >> m.offset.x;
        input >> m.offset.y;
        input >> m.offset.z;
        
        debug << "MassInfo: " << name << ": mass: " << m.mass << 
                 " offset [" << m.offset.x << ", " << m.offset.y << ", " << m.offset.z << "]" << endl;
        
        // read in the rest of the line as it is a comment
        input.ignore(128, '\n');
        return input;
    }
  };

  MassCalibration()
  {
      LoadFromFile(string(CONFIG_DIR) + string("Model/masses.cfg"));
  }
  
void LoadFromFile(const std::string& configFileName)
{
    fstream myStream(configFileName.c_str());
    
    if (myStream.is_open())
    {
        unsigned int i=0;
        while (not myStream.eof() && i<numOfLimbs)
        {
            myStream >> masses[i];
            i++;
        }
    }
    else
        debug << "MassCalibration::LoadFromFile() Failed to load masses from " << configFileName << endl;
    }
  
    MassInfo masses[numOfLimbs]; /**< Information on the mass distribution of all joints. */
};

#endif // MASSCALIBRATION_H
