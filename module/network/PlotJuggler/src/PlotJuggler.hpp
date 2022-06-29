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

        /// @brief Whether or not forwarding of DataPoint messages to PlotJuggler is enabled
        bool forward_datapoints;

        /// @brief The robot's current hostname (included in data sent, to enable filtering)
        std::string hostname;

        /// @brief The IP address of the PlotJuggler UDP server
        std::string send_address;

        /// @brief The port of the PlotJuggler UDP server
        int send_port;

        /// @brief Convert the given timepoint to a count of milliseconds since Epoch
        static inline long toMillisecondsSinceEpoch(NUClear::clock::time_point timepoint) {
            auto timepoint_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(timepoint);
            return timepoint_ms.time_since_epoch().count();
        }
    };

}  // namespace module::network

#endif  // MODULE_NETWORK_PLOTJUGGLER_HPP
