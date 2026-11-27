/*
 * MIT License
 *
 * Copyright (c) 2009 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "KeyboardSkills.hpp"

#include <Eigen/Geometry>
#include <array>
#include <clocale>
#include <cmath>
#include <csignal>
#include <fmt/format.h>
#include <iostream>

#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/input/Sensors.hpp"
#include "message/platform/NUSimBallCommand.hpp"
#include "message/skill/BallSkill.hpp"
#include "message/skill/Look.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/FallRecovery.hpp"

namespace module::purpose {

    using extension::Configuration;
    using extension::behaviour::Task;
    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::input::Sensors;
    using message::platform::NUSimBallCommand;
    using message::skill::BallSkill;
    using message::skill::Look;
    using message::skill::Walk;
    using message::strategy::FallRecovery;
    using NUClear::message::LogMessage;

    namespace {

        constexpr std::array<const char*, 7> MODE_NAMES =
            {"walk", "receive", "dribble", "kick", "air shot", "first touch", "first-time pass"};

        /// A key press, from the terminal or the script, handled in one synced reaction
        struct KeyPress {
            int key;
        };

        /// Priority of the walk and ball-skill tasks, under FallRecovery's
        constexpr int SKILL_PRIORITY = 2;

        enum Colour : short { DEFAULT = 1, DEBUG = 2, INFO = 3, WARN = 4, ERROR = 5 };

        Eigen::Vector2d vec2(const Configuration& node) {
            const auto v = node.as<std::vector<double>>();
            return v.size() == 2 ? Eigen::Vector2d(v[0], v[1]) : Eigen::Vector2d::Zero();
        }

    }  // namespace

    KeyboardSkills::KeyboardSkills(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration, Sync<KeyboardSkills>>("KeyboardSkills.yaml").then([this](const Configuration& config) {
            log_level         = config["log_level"].as<NUClear::LogLevel>();
            cfg.walk_step     = config["walk_step"].as<double>();
            cfg.turn_step     = config["turn_step"].as<double>();
            cfg.ball_step     = config["ball_step"].as<double>();
            cfg.dribble_start = vec2(config["start"]["dribble"]);
            cfg.kick_start    = vec2(config["start"]["kick"]);
            cfg.touch_start   = vec2(config["start"]["first_touch"]);
            cfg.pass_start    = vec2(config["start"]["first_pass"]);
            cfg.head_step     = config["head_step_deg"].as<double>() * M_PI / 180.0;
            cfg.pass_distance = config["pass"]["distance"].as<double>();
            cfg.pass_arrival  = config["pass"]["arrival_speed"].as<double>();
            cfg.pass_decel    = config["pass"]["rolling_deceleration"].as<double>();
            cfg.drop_distance = config["drop_distance"].as<double>();
            cfg.interactive   = config["interactive"].as<bool>();
            cfg.hold_in_world = config["hold_in_world"].as<bool>();
            cfg.script.clear();
            for (const auto& entry : config["script"].config) {
                const auto t    = entry[0].as<double>();
                const auto name = entry[1].as<std::string>();
                const int k     = name == "space" ? ' ' : name.empty() ? 0 : name[0];
                cfg.script.emplace_back(t, k);
            }
        });

        on<Startup, Sync<KeyboardSkills>>().then([this] {
            // Without these, modules that need Stability and WalkState may not run
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED));

            // The robot should always try to recover from falling
            emit<Task>(std::make_unique<FallRecovery>(), 5);

            started = NUClear::clock::now();
            if (cfg.interactive) {
                start_curses();
                terminal_reader.enable();
            }
            update_command();
            print_status();
        });

        // Read the terminal, one key at a time. getch() blocks, so this only runs with a terminal.
        terminal_reader = on<Always>().then([this] {
            const int k = getch();
            if (k != ERR) {
                emit(std::make_unique<KeyPress>(KeyPress{k}));
            }
        });

        // Scripted key presses
        on<Every<20, Per<std::chrono::seconds>>, Single, Sync<KeyboardSkills>>().then([this] {
            const double t = std::chrono::duration<double>(NUClear::clock::now() - started).count();
            while (script_next < cfg.script.size() && cfg.script[script_next].first <= t) {
                log<INFO>(fmt::format("script t={:.1f}: key '{}'",
                                      t,
                                      cfg.script[script_next].second == ' '
                                          ? std::string("space")
                                          : std::string(1, char(cfg.script[script_next].second))));
                emit(std::make_unique<KeyPress>(KeyPress{cfg.script[script_next].second}));
                ++script_next;
            }
        });

        terminal_reader.disable();

        // The heading, and a ball command held in the world turned into the robot's frame as it turns
        on<Trigger<KeyPress>, Sync<KeyboardSkills>>().then([this](const KeyPress& press) { key(press.key); });

        on<Trigger<Sensors>, Single, Sync<KeyboardSkills>>().then([this](const Sensors& sensors) {
            const Eigen::Isometry3d Hwt = Eigen::Isometry3d(sensors.Htw).inverse();
            yaw                         = std::atan2(Hwt(1, 0), Hwt(0, 0));
        });
        on<Every<25, Per<std::chrono::seconds>>, Single, Sync<KeyboardSkills>>().then([this] {
            if (mode != Mode::WALK && cfg.hold_in_world) {
                update_command();
            }
        });

        on<Trigger<LogMessage>>().then([this](const LogMessage& message) {
            if (message.level < message.display_level) {
                return;
            }
            std::string source{};
            if (message.statistics != nullptr) {
                std::string reactor = message.statistics->identifiers->reactor;
                const size_t last   = reactor.find_last_of(':');
                source              = (last == std::string::npos ? reactor : reactor.substr(last + 1)) + " ";
            }
            short colour = DEFAULT;
            switch (message.level) {
                case NUClear::LogLevel::DEBUG: colour = DEBUG; break;
                case NUClear::LogLevel::INFO: colour = INFO; break;
                case NUClear::LogLevel::WARN: colour = WARN; break;
                case NUClear::LogLevel::ERROR:
                case NUClear::LogLevel::FATAL: colour = ERROR; break;
                default: break;
            }
            if (curses) {
                write(log_window, colour, source + message.message + "\n");
            }
            else {
                // No terminal UI (and no ConsoleLogHandler in the role, which would fight the UI): plain lines
                std::lock_guard<std::mutex> lock(mutex);
                std::cerr << source << message.message << std::endl;
            }
        });

        on<Shutdown>().then([this] {
            if (curses) {
                endwin();
            }
        });
    }

    Eigen::Vector2d KeyboardSkills::ball_command_robot() const {
        return cfg.hold_in_world ? Eigen::Rotation2Dd(-yaw) * ball_command : ball_command;
    }

    void KeyboardSkills::set_ball_command_robot(const Eigen::Vector2d& command) {
        ball_command = cfg.hold_in_world ? Eigen::Rotation2Dd(yaw) * command : command;
    }

    void KeyboardSkills::key(int k) {
        const bool walking = mode == Mode::WALK;
        // Keys step the command in the robot's frame as it faces now
        const auto nudge_ball = [this](const Eigen::Vector2d& step) {
            set_ball_command_robot(ball_command_robot() + step);
        };
        switch (k) {
            case '1': set_mode(Mode::WALK); break;
            case '2': set_mode(Mode::RECEIVE); break;
            case '3': set_mode(Mode::DRIBBLE); break;
            case '4': set_mode(Mode::KICK); break;
            case '5': set_mode(Mode::KICK_AIR); break;
            case '6': set_mode(Mode::FIRST_TOUCH); break;
            case '7': set_mode(Mode::FIRST_PASS); break;
            case 'w':
                walking ? void(walk_command.x() += cfg.walk_step)
                        : nudge_ball(Eigen::Vector2d::UnitX() * (+cfg.ball_step));
                break;
            case 's':
                walking ? void(walk_command.x() -= cfg.walk_step)
                        : nudge_ball(Eigen::Vector2d::UnitX() * (-cfg.ball_step));
                break;
            case 'a':
                walking ? void(walk_command.y() += cfg.walk_step)
                        : nudge_ball(Eigen::Vector2d::UnitY() * (+cfg.ball_step));
                break;
            case 'd':
                walking ? void(walk_command.y() -= cfg.walk_step)
                        : nudge_ball(Eigen::Vector2d::UnitY() * (-cfg.ball_step));
                break;
            case 'z': walk_command.z() += cfg.turn_step; break;
            case 'x': walk_command.z() -= cfg.turn_step; break;
            case ' ':
                walk_command.setZero();
                ball_command.setZero();
                break;
            case 'p':
                roll_pass();
                // A pass coming in: receive it, unless a first touch or first-time pass is loaded for it
                if (mode != Mode::FIRST_TOUCH && mode != Mode::FIRST_PASS) {
                    set_mode(Mode::RECEIVE);
                }
                break;
            case 'b': drop_ball(); break;
            case KEY_LEFT: head_yaw += cfg.head_step; break;
            case KEY_RIGHT: head_yaw -= cfg.head_step; break;
            case KEY_UP: head_pitch += cfg.head_step; break;
            case KEY_DOWN: head_pitch -= cfg.head_step; break;
            case 'q':
                if (curses) {
                    endwin();
                    curses = false;
                }
                std::raise(SIGTERM);
                return;
            default: log<WARN>("Unknown key", k); return;
        }
        update_command();
        print_status();
    }

    void KeyboardSkills::set_mode(Mode m) {
        if (m == mode) {
            return;
        }
        mode = m;
        switch (m) {
            case Mode::WALK: walk_command.setZero(); break;
            case Mode::RECEIVE: ball_command.setZero(); break;
            case Mode::DRIBBLE: set_ball_command_robot(cfg.dribble_start); break;
            case Mode::KICK:
            case Mode::KICK_AIR: set_ball_command_robot(cfg.kick_start); break;
            case Mode::FIRST_TOUCH: set_ball_command_robot(cfg.touch_start); break;
            case Mode::FIRST_PASS: set_ball_command_robot(cfg.pass_start); break;
        }
        log<INFO>("Mode:", MODE_NAMES[int(m)]);
    }

    void KeyboardSkills::update_command() {
        if (mode == Mode::WALK) {
            emit<Task>(std::unique_ptr<BallSkill>(nullptr));
            emit<Task>(std::make_unique<Walk>(walk_command), SKILL_PRIORITY);
            // The look skill owns the head while walking; the ball skills drive it themselves
            const Eigen::Vector3d uPCt = (Eigen::AngleAxisd(head_yaw, Eigen::Vector3d::UnitZ())
                                          * Eigen::AngleAxisd(-head_pitch, Eigen::Vector3d::UnitY()))
                                             .toRotationMatrix()
                                         * Eigen::Vector3d::UnitX();
            emit<Task>(std::make_unique<Look>(uPCt, false));
        }
        else {
            emit<Task>(std::unique_ptr<Walk>(nullptr));
            auto task      = std::make_unique<BallSkill>();
            task->type     = BallSkill::Type(int(mode) - 1);
            task->velocity = ball_command_robot();
            emit<Task>(std::move(task), SKILL_PRIORITY);
        }
    }

    void KeyboardSkills::roll_pass() {
        // Launched to arrive at pass_arrival after rolling pass_distance
        const double speed = std::sqrt(cfg.pass_arrival * cfg.pass_arrival + 2.0 * cfg.pass_decel * cfg.pass_distance);
        auto command       = std::make_unique<NUSimBallCommand>();
        command->frame     = NUSimBallCommand::Frame::ROBOT;
        command->position  = Eigen::Vector3d(cfg.pass_distance, 0.0, -1.0);
        command->velocity  = Eigen::Vector3d(-speed, 0.0, 0.0);
        command->rolling   = true;
        emit(command);
        log<INFO>(fmt::format("Pass rolled from {:.1f} m at {:.2f} m/s", cfg.pass_distance, speed));
    }

    void KeyboardSkills::drop_ball() {
        auto command      = std::make_unique<NUSimBallCommand>();
        command->frame    = NUSimBallCommand::Frame::ROBOT;
        command->position = Eigen::Vector3d(cfg.drop_distance, 0.0, -1.0);
        command->velocity = Eigen::Vector3d::Zero();
        command->rolling  = true;
        emit(command);
    }

    void KeyboardSkills::start_curses() {
        std::setlocale(LC_ALL, "en_US.UTF-8");
        initscr();
        colours_enabled = has_colors();
        if (colours_enabled) {
            start_color();
            init_pair(DEFAULT, COLOR_WHITE, COLOR_BLACK);
            init_pair(DEBUG, COLOR_GREEN, COLOR_BLACK);
            init_pair(INFO, COLOR_CYAN, COLOR_BLACK);
            init_pair(WARN, COLOR_YELLOW, COLOR_BLACK);
            init_pair(ERROR, COLOR_MAGENTA, COLOR_BLACK);
        }
        cbreak();
        keypad(stdscr, true);
        noecho();
        command_window = std::shared_ptr<WINDOW>(newwin(6, COLS, 0, 0), [](WINDOW* w) { delwin(w); });
        log_window     = std::shared_ptr<WINDOW>(newwin(LINES - 6, COLS, 6, 0), [](WINDOW* w) { delwin(w); });
        scrollok(log_window.get(), true);
        curses = true;
    }

    void KeyboardSkills::print_status() {
        const std::string status = fmt::format(
            "Mode: {}  |  walk (vx, vy, wz) = ({:.2f}, {:.2f}, {:.2f})  |  ball command (vx, vy) = ({:.2f}, {:.2f}){}\n"
            "1 walk  2 receive  3 dribble  4 kick  5 air shot  6 first touch  7 first-time pass\n"
            "w/s/a/d command +/-  z/x turn (walk)  space zero  arrows head (walk)\n"
            "p roll a pass at the robot  b ball at its feet  q quit",
            MODE_NAMES[int(mode)],
            walk_command.x(),
            walk_command.y(),
            walk_command.z(),
            ball_command_robot().x(),
            ball_command_robot().y(),
            cfg.hold_in_world ? ", held in the world" : "");
        if (!curses) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex);
        werase(command_window.get());
        wmove(command_window.get(), 0, 0);
        wprintw(command_window.get(), "%s", status.c_str());
        wrefresh(command_window.get());
    }

    void KeyboardSkills::write(const std::shared_ptr<WINDOW>& window, short colour, const std::string& text) {
        std::lock_guard<std::mutex> lock(mutex);
        if (colours_enabled) {
            wattron(window.get(), COLOR_PAIR(colour));
        }
        wprintw(window.get(), "%s", text.c_str());
        if (colours_enabled) {
            wattroff(window.get(), COLOR_PAIR(colour));
        }
        wrefresh(window.get());
    }

}  // namespace module::purpose
