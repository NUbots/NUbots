#include "bind.hpp"

#include "Player.hpp"

#include "message/reflection.hpp"

namespace module::nbs {

    using utility::nbs::Decoder;

    template <typename T>
    struct BindDecoder;

    template <>
    struct BindDecoder<void> {
        virtual ~BindDecoder()                              = default;
        virtual void bind(Player& player, Decoder& decoder) = 0;
    };

    template <typename T>
    struct BindDecoder : BindDecoder<void> {
        template <typename U>
        using Inline = NUClear::dsl::word::emit::Inline<U>;
        void bind(Player& player, Decoder& decoder) override {
            decoder.on<T>([&player](const T& t) { player.emit<Inline>(std::make_unique<T>(t)); });
        }
    };

    void bind(const std::string& type, Player& player, Decoder& decoder) {
        message::reflection::from_string<BindDecoder>(type)->bind(player, decoder);
    }

}  // namespace module::nbs
