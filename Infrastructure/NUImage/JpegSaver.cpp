#include "JpegSaver.h"
#include "Tools/Image/ColorModelConversions.h"
#if defined WIN32 || defined WIN64
#include "highgui.h"
#else
#include "opencv/highgui.h"
#endif


bool JpegSaver::saveNUimageAsJpeg(NUimage* image, const std::string& pFileName)
{
    unsigned char r,g,b;
    char* bgrBuffer = new char[image->width() * image->height() * 3];
    int bufferIndex = 0;
    pixels::Pixel temp; 
    for(int y = 0; y < image->height(); y++)
    {
        for (int x = 0; x < image->width(); x++)
        {
            temp = image->image[y][x];
            ColorModelConversions::fromYCbCrToRGB(temp.y,temp.cb,temp.cr,r,g,b);
            bgrBuffer[bufferIndex++] = b;
            bgrBuffer[bufferIndex++] = g;
            bgrBuffer[bufferIndex++] = r;
        }
    }

        IplImage* fIplImageHeader;
        fIplImageHeader = cvCreateImageHeader(cvSize(image->width(), image->height()), 8, 3);
        fIplImageHeader->imageData = bgrBuffer;
        cvSaveImage(pFileName.c_str(),fIplImageHeader);
        if (fIplImageHeader)
        {
            cvReleaseImageHeader(&fIplImageHeader);
        }
        delete [] bgrBuffer;
        return true;
}
