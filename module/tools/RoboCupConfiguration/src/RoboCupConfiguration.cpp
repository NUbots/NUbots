#include "RoboCupConfiguration.hpp"

#include "extension/Configuration.hpp"

extern "C" {
#include <ncurses.h>
#undef OK
}

#include "utility/support/hostname.hpp"


namespace module::tools {

    using extension::Configuration;

    RoboCupConfiguration::RoboCupConfiguration(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("RoboCupConfiguration.yaml").then([this](const Configuration& config) {
            // Use configuration here from file RoboCupConfiguration.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            hostname        = utility::support::getHostname();
        });

        on<Startup>().then([this] {
            // Start curses mode
            initscr();
            // Capture our characters immediately (but pass through signals)
            cbreak();
            // Capture arrows and function keys
            keypad(stdscr, true);
            // Don't echo the users messages
            noecho();
            // Hide the cursor
            curs_set(0);

            refresh_view();
        });
        // When we shutdown end ncurses
        on<Shutdown>().then(endwin);
    }

    void RoboCupConfiguration::refresh_view() {
        // Clear our window
        erase();

        // Outer box
        box(stdscr, 0, 0);

        // Write our title
        attron(A_BOLD);
        mvprintw(0, (COLS - 14) / 2, " RoboCup Configuration ");
        attroff(A_BOLD);

        mvprintw(2, 2, ("Hostname: " + hostname).c_str());
        mvprintw(3, 2, ("Player ID: " + std::to_string(player_id)).c_str());
        mvprintw(4, 2, ("Team ID: " + std::to_string(team_id)).c_str());
        mvprintw(5, 2, ("IP Address: " + ip_address).c_str());
        mvprintw(6, 2, ("Position: " + robot_position).c_str());
    }


}  // namespace module::tools
