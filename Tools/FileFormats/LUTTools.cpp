#include "LUTTools.h"
#include <iostream>
#include <fstream>

bool LUTTools::LoadLUT(unsigned char* targetBuffer, int length){
    return LoadLUT( targetBuffer, length, "/home/root/default.lut");
}

bool LUTTools::LoadLUT(unsigned char* targetBuffer, int length, const char* filename){
    char* lutBuffer = (char*)targetBuffer;
    std::ifstream lutfile;
    lutfile.open(filename, std::ios::binary | std::ios::ate);

    // check if file opened correctly and is correct size
    if(lutfile.is_open() && lutfile.tellg() == LUT_SIZE)
    {
        lutfile.seekg (0, std::ios::beg);  // move to start of file.
        lutfile.read (lutBuffer, length); // read in buffer
        lutfile.close();
        return true;
    }
    else
    {
        lutfile.clear();
        return false;
    }
}

bool LUTTools::SaveLUT(unsigned char* sourceBuffer, int length){
    return SaveLUT(sourceBuffer, length, "/home/root/default.lut");
}

bool LUTTools::SaveLUT(unsigned char* sourceBuffer, int length, const char* filename){
    char* lutBuffer = (char*) sourceBuffer;
    std::fstream lutfile;
    lutfile.open(filename, std::ios::out | std::ios::binary);
    if(lutfile.is_open()){  // check if file opened correctly
        lutfile.seekp(std::ios::beg);
        lutfile.write (lutBuffer, length);
        lutfile.close();
        return true;
    } else {
        lutfile.clear();
        return false;
    }
}
