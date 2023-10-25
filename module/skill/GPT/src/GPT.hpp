#ifndef MODULE_SKILL_GPT_HPP
#define MODULE_SKILL_GPT_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::skill {

    class GPT : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief OpenAI API key
            std::string openai_api_key = "";
            /// @brief Device name to record from
            std::string device_name = "default";
            /// @brief Pre-prompt for GPT request
            std::string pre_prompt = "";
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the GPT reactor.
        explicit GPT(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_GPT_HPP
