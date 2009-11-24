/*!
  @file LUTTools.h
  @author Steven Nicklin
  @brief Defines some files used to load and save a lookup table to file.
*/
#ifndef LUTTOOLS_H_DEFINED
#define LUTTOOLS_H_DEFINED

/*!
  @brief Class contains functions used to load a colour lookup table from a file and also save a colour
         lookup table to a file.
  */
class LUTTools
{
public:
    static const int LUT_SIZE = 256*256*256; //!< The size of a lookup table in bytes.

    /*!
      @brief Load a lookup table from a default file into a supplied buffer.
      @param targetBuffer The buffer to which the colour lookup table will be written.
      @param length The length of the colour lookuptable in bytes.
      @return True if the file was loaded into the buffer sucessfully. False if it was not.
      */
    static bool LoadLUT(unsigned char* targetBuffer, int length);
    /*!
      @brief Load a lookup table from a specified file into a supplied buffer.
      @param targetBuffer The buffer to which the colour lookup table will be written.
      @param length The length of the colour lookuptable in bytes.
      @param fileName The name of the file to be loaded.
      @return True if the file was loaded into the buffer sucessfully. False if it was not.
      */
    static bool LoadLUT(unsigned char* targetBuffer, int length, const char* filename);
    /*!
      @brief Save a lookup table from a supplied buffer into the default file.
      @param sourceBuffer The buffer in which the colour lookup table is stored.
      @param length The length of the colour lookuptable in bytes.
      @return True if the colour lookup table was saved successfully. False if it was not.
      */
    static bool SaveLUT(unsigned char* sourceBuffer, int length);
    /*!
      @brief Save a lookup table from a supplied buffer into the default file.
      @param sourceBuffer The buffer in which the colour lookup table is stored.
      @param length The length of the colour lookuptable in bytes.
      @param fileName The name of the file to save the colour lookup table.
      @return True if the colour lookup table was saved successfully. False if it was not.
      */
    static bool SaveLUT(unsigned char* sourceBuffer, int length, const char* filename);
};
#endif
