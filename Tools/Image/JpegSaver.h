#ifndef __JPEGSAVER_H__
#define __JPEGSAVER_H__

#include "NUimage.h"
#include <string>

class JpegSaver
{
public:
    static bool saveNUimageAsJpeg(NUimage* image, const std::string& pFileName);
};

#endif
