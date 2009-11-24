#include "LUTTools.h"
#include <iostream>
#include <fstream>

using namespace std;

bool LUTTools::LoadLUT(unsigned char* targetBuffer, int length){
    return LoadLUT( targetBuffer, length, "/home/root/default.lut");
}

bool LUTTools::LoadLUT(unsigned char* targetBuffer, int length, const char* filename){
    char* lutBuffer = (char*)targetBuffer;
    fstream lutfile; 
    lutfile.open(filename, ios::in | ios::binary);
    if(lutfile.is_open()){  // check if file opened correctly
        lutfile.seekg (0, ios::beg);  // move to start of file.
        lutfile.read (lutBuffer, length); // read in buffer
        lutfile.close();
        return true;
    } else {
        lutfile.clear();
        return false;
    }
}

bool LUTTools::SaveLUT(unsigned char* sourceBuffer, int length){
    return SaveLUT(sourceBuffer, length, "/home/root/default.lut");
}

bool LUTTools::SaveLUT(unsigned char* sourceBuffer, int length, const char* filename){
    char* lutBuffer = (char*) sourceBuffer;
    fstream lutfile;
    lutfile.open(filename, ios::out | ios::binary);
    if(lutfile.is_open()){  // check if file opened correctly
        lutfile.seekp(ios::beg);
        lutfile.write (lutBuffer, length);
        lutfile.close();
        return true;
    } else {
        lutfile.clear();
        return false;
    }
}
