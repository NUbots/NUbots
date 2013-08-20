#ifndef MESSAGES_IMAGE_H
#define MESSAGES_IMAGE_H

#include <memory>

namespace messages {
    
    class Image {
    public:    
        struct Pixel {
            private:
                uint8_t padding;
            public:
                uint8_t cb;
                uint8_t y;
                uint8_t cr;
        };
        
        Image(size_t width, size_t height, std::unique_ptr<Pixel[]>&& data);
        Pixel& operator()(size_t x, size_t y);
        
        const Pixel& operator()(size_t x, size_t y) const;
        const size_t width() const;
        const size_t height() const;
        const size_t size() const;
        
    private:
        size_t imgWidth;
        size_t imgHeight;
        std::unique_ptr<Pixel[]> data;
    };
}
#endif
