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
        using messages::vision::ClassifiedImage;
        
        LookUpTable::LookUpTable() {
            LUTbuffer = new unsigned char[LUT_SIZE];
			
            for(int i = 0; i < LUT_SIZE; i++) {
                LUTbuffer[i] = ClassifiedImage::unclassified;
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
                LUTbuffer[i] = ClassifiedImage::unclassified;
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

        const ClassifiedImage::Colour LookUpTable::getColourFromIndex(int index) const {
            switch (index) {
                case 0: {
					return ClassifiedImage::unclassified;
                }

                case 1: {
					return ClassifiedImage::white;
                }

                case 2: {
					return ClassifiedImage::green;
                }

                case 3: {
					return ClassifiedImage::shadow_object;
                }

                case 4: {
					return ClassifiedImage::pink;
                }

                case 5: {
					return ClassifiedImage::pink_orange;
                }

                case 6: {
					return ClassifiedImage::orange;
                }

                case 7: {
					return ClassifiedImage::yellow_orange;
                }

                case 8: {
					return ClassifiedImage::yellow;
                }

                case 9: {
					return ClassifiedImage::blue;
                }

                case 10: { 
					return ClassifiedImage::shadow_blue;
                }

                default: {
					return ClassifiedImage::invalid;
                }
            }
        }

        std::string LookUpTable::getColourName(const ClassifiedImage::Colour& colour) {
            switch (colour) {
                case ClassifiedImage::unclassified: {
                    return "unclassified";
                }

                case ClassifiedImage::white: {
                    return "white";
                }

                case ClassifiedImage::green: {
                    return "green";
                }

                case ClassifiedImage::shadow_object: {
                    return "shadow object";
                }

                case ClassifiedImage::pink: {
                    return "pink";
                }

                case ClassifiedImage::pink_orange: {
                    return "pink - orange";
                }

                case ClassifiedImage::orange: {
                    return "orange";
                }

                case ClassifiedImage::yellow_orange: {
                    return "yellow - orange";
                }

                case ClassifiedImage::yellow: {
                    return "yellow";
                }

                case ClassifiedImage::blue: {
                    return "blue";
                }

                case ClassifiedImage::shadow_blue: {
                    return "shadow blue";
                }

                default: {
                    return "unknown colour!";
                }
            }
        }
		
		ClassifiedImage::Colour LookUpTable::getColourFromName(const std::string& name) {
			if (name.compare("unclassified") == 0) {
				return ClassifiedImage::unclassified;
			}

			else if (name.compare("white") == 0) {
				return ClassifiedImage::white;
			}

			else if (name.compare("green") == 0) {
				return ClassifiedImage::green;
			}

			else if (name.compare("shadow object") == 0) {
				return ClassifiedImage::shadow_object;
			}

			else if (name.compare("pink") == 0) {
				return ClassifiedImage::pink;
			}

			else if (name.compare("pink - orange") == 0) {
				return ClassifiedImage::pink_orange;
			}

			else if (name.compare("orange") == 0) {
				return ClassifiedImage::orange;
			}

			else if (name.compare("yellow - orange") == 0) {
				return ClassifiedImage::yellow_orange;
			}

			else if (name.compare("yellow") == 0) {
				return ClassifiedImage::yellow;
			}

			else if (name.compare("blue") == 0) {
				return ClassifiedImage::blue;
			}

			else if (name.compare("shadow blue") == 0) {
				return ClassifiedImage::shadow_blue;
			}

			else {
				return ClassifiedImage::invalid;
			}
		}
		
    }   //vision
}   //modules