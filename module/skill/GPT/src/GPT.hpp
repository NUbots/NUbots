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
            /// @brief Model
            std::string model = "gpt-3.5-turbo";
            /// @brief Maximum tokens to generate
            int max_tokens = 100;
            /// @brief Temperature of the model
            double temperature = 0;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the GPT reactor.
        explicit GPT(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_GPT_HPP
