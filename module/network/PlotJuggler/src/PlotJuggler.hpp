#ifndef MODULE_NETWORK_PLOTJUGGLER_HPP
#define MODULE_NETWORK_PLOTJUGGLER_HPP

#include <nuclear>
#include <string>

namespace module::network {

    class PlotJuggler : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the PlotJuggler reactor.
        explicit PlotJuggler(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// @brief The reactor start time in milliseconds since Epoch.
        /// Used to normalise timestamps in forwarded DataPoint messages.
        long start_time_ms;

        /// @brief The robot's current hostname (included in data sent, to enable filtering)
        std::string hostname;

        /// @brief The IP address of the PlotJuggler UDP server
        std::string send_address;

        /// @brief The port of the PlotJuggler UDP server
        int send_port;

        /// @brief Handle to the reaction that forwards DataPoints to PlotJuggler
        ReactionHandle forwarder_reaction{};

        /// @brief Handle to the reaction that generates debug waves for testing PlotJuggler connection
        ReactionHandle debug_waves_reaction{};

        /// @brief Convert the given timepoint to a count of milliseconds since Epoch
        static inline long toMillisecondsSinceEpoch(NUClear::clock::time_point timepoint) {
            return std::chrono::time_point_cast<std::chrono::milliseconds>(timepoint).time_since_epoch().count();
        }
    };

}  // namespace module::network

#endif  // MODULE_NETWORK_PLOTJUGGLER_HPP
