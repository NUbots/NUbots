#ifndef MODULE_PURPOSE_CHAT_HPP
#define MODULE_PURPOSE_CHAT_HPP

#include <Eigen/Core>
#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::purpose {

    class Chat : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {

            /// @brief User prompt
            std::string prompt = "";

            /// @brief OpenAI API key
            std::string openai_api_key = "";
        } cfg;


    public:
        /// @brief Called by the powerplant to build and setup the CHAT reactor.
        explicit Chat(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_CHAT_HPP
