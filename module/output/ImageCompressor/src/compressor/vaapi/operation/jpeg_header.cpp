#include "jpeg_header.h"

#include <cstring>
#include <vector>

#include "../vaapi_error_category.hpp"
#include "jpeg_constants.h"

// Included for htons!
#include <arpa/inet.h>

namespace module::output::compressor::vaapi::operation {

#pragma pack(push, 1)
namespace markers {

    /// htons is used throughout the code to fix the endian issue
    /// In JPEG all of the 16 bit numbers are big endian, conveniently this is the same byte order as "network order"
    /// So we can use host to network short to ensure that the endianess of the bytes is correct

    // Start of image
    struct SOI {
        uint8_t marker = 0xFF;
        uint8_t type   = 0xD8;
    };

    // Application Data
    struct AppData {
        uint8_t marker             = 0xFF;
        uint8_t type               = 0xE0;  // APP0 Marker
        uint16_t length            = htons(sizeof(AppData) - 2);
        std::array<char, 5> header = {'J', 'F', 'I', 'F', 0x00};
        uint8_t major              = 1;
        uint8_t minor              = 1;
        uint8_t density            = 1;
        uint16_t x_density         = htons(72);
        uint16_t y_density         = htons(72);
        uint8_t thumbnail_width    = 0;
        uint8_t thumbnail_height   = 0;
    };

    // Define quantisation table
    struct DQT {
        uint8_t marker             = 0xFF;
        uint8_t type               = 0xDB;
        uint16_t length            = htons(sizeof(DQT) - 2);
        uint8_t q_precision_q_type = 0;
        std::array<uint8_t, 64> quant;

        DQT(const std::array<uint8_t, 64>& table, int quality) {
            // Normalise by quality and clamp between 1 and 255
            quality = (quality < 50) ? (5000 / quality) : (200 - (quality * 2));
            for (size_t i = 0; i < table.size(); ++i) {
                int v    = (table[jpeg_zigzag[i]] * quality) / 100;
                quant[i] = std::max(1, std::min(255, v));
            }
        }
    };

    // Define huffman table
    template <int Entries>
    struct DHT {
        uint8_t marker  = 0xFF;
        uint8_t type    = 0xC4;
        uint16_t length = htons(sizeof(DHT<Entries>) - 2);

        // In a huffman table, there is 1 entry that describes the type of table (ac/dc etc)
        // Then there are 16 values that say how many table entries of each byte size there are
        // Following this there are those entries that were described in those 16 bytes
        std::array<uint8_t, 1 + 16 + Entries> table;

        DHT(const std::array<uint8_t, 1 + 16 + Entries>& table) : table(table) {}
    };

    struct SOS_Monochrome {
        uint8_t marker       = 0xFF;
        uint8_t type         = 0xDA;
        uint16_t length      = htons(sizeof(SOS_Monochrome) - 2);
        uint8_t components   = 1;
        uint8_t y_id         = 1;
        uint8_t y_huff_table = 0x00;
        uint8_t start        = 0x00;
        uint8_t end          = 0x3F;
        uint8_t sp           = 0x00;
    };

    struct SOS_Colour {
        uint8_t marker       = 0xFF;
        uint8_t type         = 0xDA;
        uint16_t length      = htons(sizeof(SOS_Colour) - 2);
        uint8_t components   = 3;
        uint8_t y_id         = 1;
        uint8_t y_huff_table = 0x00;
        uint8_t u_id         = 2;
        uint8_t u_huff_table = 0x11;
        uint8_t v_id         = 2;
        uint8_t v_huff_table = 0x11;
        uint8_t start        = 0x00;
        uint8_t end          = 0x3F;
        uint8_t sp           = 0x00;
    };

    // Start of frame
    struct SOF0_Monochrome {
        uint8_t marker    = 0xFF;
        uint8_t type      = 0xC0;
        uint16_t length   = htons(sizeof(SOF0_Monochrome) - 2);
        uint8_t precision = 8;
        uint16_t height;
        uint16_t width;
        uint8_t n_components = 1;

        uint8_t y_id             = 1;
        uint8_t y_samp           = 0x11;
        uint8_t y_quant_table_no = 0;

        SOF0_Monochrome(uint16_t width, uint16_t height) : height(htons(height)), width(htons(width)) {}
    };

    struct SOF0_Colour {
        uint8_t marker    = 0xFF;
        uint8_t type      = 0xC0;
        uint16_t length   = htons(sizeof(SOF0_Colour) - 2);
        uint8_t precision = 8;
        uint16_t height;
        uint16_t width;
        uint8_t n_components = 3;

        uint8_t y_id             = 1;
        uint8_t y_samp           = 0x11;
        uint8_t y_quant_table_no = 0;

        uint8_t u_id             = 2;
        uint8_t u_samp           = 0x11;
        uint8_t u_quant_table_no = 1;

        uint8_t v_id             = 3;
        uint8_t v_samp           = 0x11;
        uint8_t v_quant_table_no = 1;

        SOF0_Colour(uint16_t width, uint16_t height) : height(height), width(width) {}
    };

}  // namespace markers
#pragma pack(pop)

template <typename T, typename... Args>
void emplace_bytes(std::vector<uint8_t>& v, Args&&... args) {
    size_t start = v.size();
    v.resize(v.size() + sizeof(T));
    new (v.data() + start) T(std::forward<Args>(args)...);
}

std::pair<VABufferID, VABufferID> jpeg_header(VADisplay dpy,
                                              VAContextID context,
                                              uint32_t width,
                                              uint32_t height,
                                              const bool& monochrome,
                                              int quality) {

    /// This jpeg header builder works using a c++ technique called placement new
    /// https://en.cppreference.com/w/cpp/language/new#Placement_new
    /// In placement new, the memory allocation is separated from the object construction.
    /// By using this we can take our header as an std::vector, and continuously increase it's size and construct new
    /// objects on it using placement new.
    /// By doing this we can define easy to work with packed structs that place the data in the correct location

    // The final location for header data
    std::vector<uint8_t> header;

    // Add the Start of image segment
    emplace_bytes<markers::SOI>(header);

    // Add the Application Data segment
    emplace_bytes<markers::AppData>(header);

    // Add the luma quantization table
    emplace_bytes<markers::DQT>(header, jpeg_luma_quant, quality);

    // If colour add the chroma quantisation table
    if (!monochrome) {
        emplace_bytes<markers::DQT>(header, jpeg_chroma_quant, quality);
    }

    // Depending on if we have a monochrome image, add the correct start of frame header
    if (monochrome) {
        emplace_bytes<markers::SOF0_Monochrome>(header, uint16_t(width), uint16_t(height));
    }
    else {
        emplace_bytes<markers::SOF0_Colour>(header, uint16_t(width), uint16_t(height));
    }

    // Add the dc and ac luma huffman table
    emplace_bytes<markers::DHT<12>>(header, jpeg_hufftable_luma_dc);
    emplace_bytes<markers::DHT<162>>(header, jpeg_hufftable_luma_ac);

    // If colour add the chroma huffman table
    if (!monochrome) {
        emplace_bytes<markers::DHT<12>>(header, jpeg_hufftable_chroma_dc);
        emplace_bytes<markers::DHT<162>>(header, jpeg_hufftable_chroma_ac);
    }

    // Add the scan header
    if (monochrome) {
        emplace_bytes<markers::SOS_Monochrome>(header);
    }
    else {
        emplace_bytes<markers::SOS_Colour>(header);
    }

    VAEncPackedHeaderParameterBuffer params;
    params.type                = VAEncPackedHeaderRawData;
    params.bit_length          = header.size() * 8;
    params.has_emulation_bytes = 0;

    VABufferID paramid;
    VAStatus va_status = vaCreateBuffer(dpy,
                                        context,
                                        VAEncPackedHeaderParameterBufferType,
                                        sizeof(VAEncPackedHeaderParameterBuffer),
                                        1,
                                        &params,
                                        &paramid);
    if (va_status != VA_STATUS_SUCCESS) {
        throw std::system_error(va_status, vaapi_error_category(), "Error creating the raw header parameter buffer");
    }

    VABufferID rawid;
    va_status = vaCreateBuffer(dpy, context, VAEncPackedHeaderDataBufferType, header.size(), 1, header.data(), &rawid);
    if (va_status != VA_STATUS_SUCCESS) {
        throw std::system_error(va_status, vaapi_error_category(), "Error creating the raw header buffer");
    }

    return std::make_pair(paramid, rawid);
}

}  // namespace module::output::compressor::vaapi::operation
