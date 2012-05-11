#include "lookuptable.h"
#include "debug.h"
#include "debugverbosityvision.h"
#include "Vision/VisionTools/classificationcolours.h"

LookUpTable::LookUpTable()
{
    LUTbuffer = new unsigned char[LUTTools::LUT_SIZE];
}

bool LookUpTable::loadLUTFromFile(const string& fileName)
{
    LUTTools loader;
    bool load_success;
    load_success = loader.LoadLUT(LUTbuffer, LUTTools::LUT_SIZE,fileName.c_str());
    if(load_success) {
        LUT = LUTbuffer;
    }
    else {
        errorlog << "Vision::loadLUTFromFile(" << fileName << "). Failed to load lut." << endl;
    }

#ifdef DEBUG_VISION_VERBOSITY_ON
    if(load_success)
    {
        debug << "Lookup table: " << fileName << " loaded sucesfully." << std::endl;
    }
    else
    {
        debug << "Unable to load lookup table: " << fileName << std::endl;
    }
#endif

    return load_success;
}

void LookUpTable::classifyImage(const NUImage& src, Mat& dest) const
{
    int width = src.getWidth();
    int height = src.getHeight();
    unsigned char r, g, b;

    dest.create(height, width, CV_8UC3);

    for (int y = 0; y < height; y++)
    {
        unsigned char* row = dest.ptr<unsigned char>(y);
        for (int x = 0; x < width; x++)
        {
            ClassIndex::getColourIndexAsRGB((int)classifyPixel(src(x,y)), r, g, b);
            row[3*x] = b;
            row[3*x+1] = g;
            row[3*x+2] = r;
        }
    }
}
