#ifndef MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_TURBOJPEG_FACTORY_HPP
#define MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_TURBOJPEG_FACTORY_HPP

#include "../DecompressorFactory.hpp"
#include "Decompressor.hpp"

namespace module::input::decompressor::turbojpeg {

    class Factory : public DecompressorFactory {
    public:
        Factory();
        Factory(const Factory&) = default;
        Factory(Factory&&)      = default;
        Factory& operator=(const Factory&) = default;
        Factory& operator=(Factory&&) = default;
        virtual ~Factory();

        std::shared_ptr<decompressor::Decompressor> make_decompressor(const uint32_t& width,
                                                                      const uint32_t& height,
                                                                      const uint32_t& format) override;
    };

}  // namespace module::input::decompressor::turbojpeg

#endif  //  MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_TURBOJPEG_FACTORY_HPP
