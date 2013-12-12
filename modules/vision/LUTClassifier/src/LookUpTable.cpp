/**
*       @name LookUpTable
*       @file lookuptable.cpp
*       @brief Wraps LUT buffer with access methods for pixel classification
*       @author Shannon Fenn
*       @date 17-02-12
*       @note ported by Jake Fountain Dec 2013 to NUClear system
*/

#include "LookUpTable.h"

namespace modules{
  namespace vision{

        using messages::vision;
        
        LookUpTable::LookUpTable() {
            LUTbuffer = new unsigned char[LUT_SIZE];
			
            for(int i = 0; i < LUT_SIZE; i++) {
                LUTbuffer[i] = unclassified;
			}
			
            LUT = LUTbuffer;
        }

        LookUpTable::LookUpTable(unsigned char *vals) {
            LUTbuffer = new unsigned char[LUT_SIZE];
            set(vals);
        }

        void LookUpTable::set(unsigned char *vals) {
            for(int i = 0; i < LUT_SIZE; i++) {
                LUTbuffer[i] = vals[i];
            }
			
            LUT = LUTbuffer;
        }

        bool LookUpTable::loadLUTFromFile(const std::string& file_name) {
            bool load_success;
            char* lutBuffer = (char*)LUTbuffer;
            std::ifstream lutfile;
            lutfile.open(file_name, std::ios::binary | std::ios::ate);

            // check if file opened correctly and is correct size
            if ((lutfile.is_open()) && (lutfile.tellg() == LUT_SIZE)) {
                lutfile.seekg (0, std::ios::beg);  // move to start of file.
                lutfile.read (lutBuffer, LUT_SIZE); // read in buffer
                lutfile.close();
                load_success = true;
            }
			
            else {
                lutfile.clear();
                load_success = false;
            }


            if(load_success) {
                LUT = LUTbuffer;
            }
			
            else {
                //log<NUClear::DEBUG>("Vision::loadLUTFromFile(", file_name, "). Failed to load lut.");
				std::cout << "Vision::loadLUTFromFile(" << file_name << "). Failed to load lut." << std::endl;
            }
			
            return load_success;
        }

        void LookUpTable::zero() {
            for(int i = 0; i < LUT_SIZE; i++) {
                LUTbuffer[i] = unclassified;
			}
			
            LUT = LUTbuffer;
        }

        const unsigned int LookUpTable::getLUTIndex(const messages::input::Image::Pixel& colour) const {
            unsigned int index = 0;
			
            index += ((colour.y >> 1) << 14);
            index += ((colour.cb >> 1) << 7);
            index += (colour.cr >> 1);
			
            return index;
        }
		
    }   //vision
}   //modules