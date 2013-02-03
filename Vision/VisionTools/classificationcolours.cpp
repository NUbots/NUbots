#include "classificationcolours.h"

namespace Vision {
    /*!
      Converts the integer index into the specified colour index.
      @param index The index of the colour desired.
      @return The colour of the given index.
      */
    Colour getColourFromIndex(int index)
    {
        switch(index)
        {
            case 0:  return unclassified;
            case 1:  return white;
            case 2:  return green;
            case 3:  return shadow_object;
            case 4:  return pink;
            case 5:  return pink_orange;
            case 6:  return orange;
            case 7:  return yellow_orange;
            case 8:  return yellow;
            case 9:  return blue;
            case 10: return shadow_blue;
            default: return invalid;
        };
    }

    /*!
      Gets the name of the given colour.
      @param colour The colour name desired.
      @return The name of the colour.
      */
    std::string getColourName(Colour colour)
    {
        switch(colour)
        {
            case unclassified:  return "unclassified";
            case white:         return "white";
            case green:         return "green";
            case shadow_object: return "shadow object";
            case pink:           return "pink";
            case pink_orange:    return "pink - orange";
            case orange:        return "orange";
            case yellow_orange: return "yellow - orange";
            case yellow:        return "yellow";
            case blue:          return "blue";
            case shadow_blue:   return "shadow blue";
            default:            return "unknown colour!";
        };
    }

    /*!
      Gets the colour matching given name.
      @param name String name of the colour desired.
      @return The method mathing the given name.
      */
    Colour getColourFromName(const std::string& name)
    {
        if(name.compare("unclassified") == 0)
            return unclassified;
        else if(name.compare("white") == 0)
            return white;
        else if(name.compare("green") == 0)
            return green;
        else if(name.compare("shadow object") == 0)
            return shadow_object;
        else if(name.compare("pink") == 0)
            return pink;
        else if(name.compare("pink - orange") == 0)
            return pink_orange;
        else if(name.compare("orange") == 0)
            return orange;
        else if(name.compare("yellow - orange") == 0)
            return yellow_orange;
        else if(name.compare("yellow") == 0)
            return yellow;
        else if(name.compare("blue") == 0)
            return blue;
        else if(name.compare("shadow blue") == 0)
            return shadow_blue;
        else
            return invalid;
    }

    /*!
      Gets the RGB representation of the colour represented by the given index.
      @param colour The colour whos RGB is required.
      @param r the target R value to which the result will be written.
      @param g the target G value to which the result will be written.
      @param b the target B value to which the result will be written.
      */
    void getColourAsRGB(Colour colour, unsigned char &r, unsigned char &g, unsigned char &b)
    {
        #define RGB(_r,_g,_b) r=_r; g=_g; b=_b; return
        switch(colour)
        {
            case unclassified:  RGB(0,0,0);
            case white:         RGB(255,255,255);
            case green:         RGB(0,255,0);
            case shadow_object: RGB(168,168,168);
            case pink:          RGB(255,20,127);
            case pink_orange:   RGB(255,128,128);
            case orange:        RGB(255,165,0);
            case yellow_orange: RGB(238,219,83);
            case yellow:        RGB(255,255,0);
            case blue:          RGB(0,0,255);
            case shadow_blue:   RGB(25,25,112);
            default:            RGB(0,0,0);
        };
        #undef RGB
    }
}
