#include "LookUpTable.h"
#include <fstream>
#include <cstring>

namespace messages {
    namespace vision {

        void LookUpTable::loadLUTFromFile(const std::string& fileName) {
            std::ifstream lutfile(fileName, std::ios::binary);
            std::copy(std::istreambuf_iterator<char>(lutfile), std::istreambuf_iterator<char>(), std::begin(LUT));
        }

        void LookUpTable::loadLUTFromArray(const char* array) {
            std::memcpy(&LUT, array, LUT_SIZE);
        }

        messages::vision::Colour LookUpTable::classifyPixel(const messages::input::Image::Pixel& p) const {
            return messages::vision::Colour(LUT[getLUTIndex(p)]); // 7bit LUT
        }

        uint LookUpTable::getLUTIndex(const messages::input::Image::Pixel& colour) {
            unsigned int index = 0;

            index += ((colour.y >> (8 - BITS_PER_COLOUR)) << 2 * BITS_PER_COLOUR);
            index += ((colour.cb >> (8 - BITS_PER_COLOUR)) << BITS_PER_COLOUR);
            index += (colour.cr >> (8 - BITS_PER_COLOUR));

            return index;
        }

    } //vision
} // messages
