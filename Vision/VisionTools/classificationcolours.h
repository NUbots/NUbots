/*!
  @file ClassificationColours.h
  @author Steven Nicklin
  @brief A class containing information about the colour classification categories.

    Colour classification is used to group colours found in an image into common
    categories that we can use to help distinguish between different coloured objects. Included
    are the common colours found in the field objects that we are interested in, as well as
    'Soft' colours used to define regions that cannot be directly mapped into one particular
    category, but instead specify that the colour may belong to one of two categories.

    Functions are available to get details about a particular colour classification.
    This information includes the name of the category using getColourName() and the colour that
    should be used to display the colour category using getColourAsRGB().

  @note Modified by Shannon Fenn on 24/03/12

    Added:
        - getColourFromName method
        - extra enum value of invalid (not counted in num_colours), needed for previous method
  */

#ifndef CLASSIFICATIONCOLOURS_H
#define CLASSIFICATIONCOLOURS_H

#include <string>
#include <iostream>

namespace Vision
{
    /*!
      @enum Colour Classified colour indicies used to identify the colour category of the
      pixels in the image.
      */
    enum Colour
    {
        unclassified,   //!< Colour has not be given a category.
        white,          //!< Colour is in the White region.
        green,          //!< Colour is in the Green region.
        shadow_object,  //!< Colour is part of a shadowed area.
        pink,            //!< Colour is in the Red region.
        pink_orange,     //!< Colour is in the region of overlap between Red and Orange.
        orange,         //!< Colour is in the Orange region.
        yellow_orange,  //!< Colour is in the region of overlap between Yellow and Orange.
        yellow,         //!< Colour is in the Yellow region.
        blue,           //!< Colour is in the Sky Blue region.
        shadow_blue,    //!< Colour is in the Dark Blue region.
        num_colours,    //!< Total number of colour categories.
        invalid
    };

    /*!
      Converts the integer index into the specified colour index.
      @param index The index of the colour desired.
      @return The colour of the given index.
      */
    Colour getColourFromIndex(int index);

    /*!
      Gets the name of the given colour.
      @param colour The colour name desired.
      @return The name of the colour.
      */
    std::string getColourName(Colour colour);

    /*!
      Gets the colour matching given name.
      @param name String name of the colour desired.
      @return The method mathing the given name.
      */
    Colour getColourFromName(const std::string& name);

    /*!
      Gets the RGB representation of the colour represented by the given index.
      @param colour The colour whos RGB is required.
      @param r the target R value to which the result will be written.
      @param g the target G value to which the result will be written.
      @param b the target B value to which the result will be written.
      */
    void getColourAsRGB(Colour colour, unsigned char &r, unsigned char &g, unsigned char &b);
}

#endif // CLASSIFICATIONCOLOURS_H
