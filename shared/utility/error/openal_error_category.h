#ifndef UTILITY_ERROR_OPENAL_ERROR_CATEGORY_H
#define UTILITY_ERROR_OPENAL_ERROR_CATEGORY_H
#include <system_error>
#include <string>
#include <AL/al.h>

namespace utility {
namespace error {
    enum class openal_errorc {
        NO_ERROR,
        INVALID_NAME,
        INVALID_ENUM,
        INVALID_OPERATION,
        OUT_OF_MEMORY,
        UNKNOWN
    };
}
}

namespace std {
    template<> struct is_error_condition_enum<utility::error::openal_errorc> : public true_type {};
}

namespace utility {
namespace error {

    class openal_error_category_t : public std::error_category {
        public:

            virtual const char* name() const noexcept;

            virtual std::error_condition default_error_condition(int code) const noexcept;

            virtual bool equivalent(const std::error_code& code, int condition) const noexcept;

            virtual std::string message(int code) const noexcept;
    };

    const std::error_category& openal_error_category();

    std::error_condition make_error_condition(utility::error::openal_errorc e);
}
}

namespace utility {
namespace error {
    const std::error_category& openal_error_category() {
        static openal_error_category_t instance;
        return instance;
    }

    std::error_condition make_error_condition(utility::error::openal_errorc e) {
        return std::error_condition(static_cast<int>(e), utility::error::openal_error_category());
    }

    const char* openal_error_category_t::name() const noexcept {
        return "openal_error_category";
    }

    std::error_condition openal_error_category_t::default_error_condition(int code) const noexcept {
        switch(code) {
            case AL_NO_ERROR: return std::error_condition(openal_errorc::NO_ERROR);
            case AL_INVALID_NAME: return std::error_condition(openal_errorc::INVALID_NAME);
            case AL_INVALID_ENUM: return std::error_condition(openal_errorc::INVALID_ENUM);
            case AL_INVALID_OPERATION: return std::error_condition(openal_errorc::INVALID_OPERATION);
            case AL_OUT_OF_MEMORY: return std::error_condition(openal_errorc::OUT_OF_MEMORY);
            default: return std::error_condition(openal_errorc::UNKNOWN);
        }
    }

    bool openal_error_category_t::equivalent(const std::error_code& code, int condition) const noexcept {
        return *this == code.category() &&
            static_cast<int>(default_error_condition(code.value()).value()) == condition;
    }

    std::string openal_error_category_t::message(int code) const noexcept {
        switch(code) {
            case AL_NO_ERROR: return "No error.";
            case AL_INVALID_NAME: return "Invalid Name parameter passed to AL call.";
            case AL_INVALID_ENUM: return "Invalid enum parameter value.";
            case AL_INVALID_OPERATION: return "Illegal call.";
            case AL_OUT_OF_MEMORY: return "Out of memory";
            default: return "Unknown error";
        }
    } 
}
}
#endif
