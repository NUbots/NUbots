#ifndef MODULE_NBS_PLAYER_BIND_HPP
#define MODULE_NBS_PLAYER_BIND_HPP

#include <cstddef>
#include <cstdint>

#include "utility/nbs/Decoder.hpp"

namespace module::nbs {

    class Player;

    /**
     * Emits data through the player instance
     *
     * @param type Name of the type to bind
     * @param player Reference to the Player instance that will handle the emission
     * @param decoder Reference to the Decoder instance that will handle the emission
     */
    void bind(const std::string& type, Player& player, utility::nbs::Decoder& decoder);

}  // namespace module::nbs

#endif  // MODULE_NBS_PLAYER_BIND_HPP
