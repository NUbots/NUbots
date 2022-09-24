#include "SystemConfiguration.hpp"

#include <filesystem>
#include <fmt/format.h>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <unistd.h>

#include "glyphs.hpp"

#include "extension/Configuration.hpp"

namespace module::tools {

    using extension::Configuration;
    namespace fs = std::filesystem;

    [[nodiscard]] bool files_equal(const fs::path& a, const fs::path& b) {
        // If both files exist and are the same size
        if (fs::is_regular_file(a) && fs::is_regular_file(b) && fs::file_size(a) == fs::file_size(b)) {
            std::ifstream a_f(a);
            std::ifstream b_f(b);
            return std::equal(std::istreambuf_iterator<char>(a_f),
                              std::istreambuf_iterator<char>(),
                              std::istreambuf_iterator<char>(b_f));
        }
        return false;
    }

    [[nodiscard]] std::string permissions_to_string(const fs::perms& p) {
        return fmt::format("{}{}{}{}{}{}{}{}{}",
                           ((p & fs::perms::owner_read) != fs::perms::none ? "r" : "-"),
                           ((p & fs::perms::owner_write) != fs::perms::none ? "w" : "-"),
                           ((p & fs::perms::owner_exec) != fs::perms::none ? "x" : "-"),
                           ((p & fs::perms::group_read) != fs::perms::none ? "r" : "-"),
                           ((p & fs::perms::group_write) != fs::perms::none ? "w" : "-"),
                           ((p & fs::perms::group_exec) != fs::perms::none ? "x" : "-"),
                           ((p & fs::perms::others_read) != fs::perms::none ? "r" : "-"),
                           ((p & fs::perms::others_write) != fs::perms::none ? "w" : "-"),
                           ((p & fs::perms::others_exec) != fs::perms::none ? "x" : "-"));
    }

    [[nodiscard]] std::string big_text(const std::string& text) {
        std::stringstream output;
        for (int row = 0; row < 8; ++row) {
            for (const char& c : text) {
                output << glyphs[row][int(c)];
            }
            output << "\n";
        }
        return output.str();
    }

    void symlink(const fs::path& link, const fs::path& target) {
        NUClear::log<NUClear::TRACE>(fmt::format("Checking link {} -> {}", link.string(), target.string()));
        if (!fs::exists(target)) {
            NUClear::log<NUClear::WARN>(fmt::format("Link target '{}' doesn't exist. Skipping", target.string()));
        }
        else if (fs::exists(link)) {
            if (!fs::is_symlink(link)) {
                NUClear::log<NUClear::WARN>(
                    fmt::format("File '{}' already exists but is not a symlink. Deleting it", link.string()));

                // Backup the old file and the delete the conflict
                fs::path new_link = link;
                new_link += ".old";
                fs::copy_file(link, new_link);
                fs::remove(link);
            }
            else if (fs::read_symlink(link).compare(target) != 0) {
                NUClear::log<NUClear::WARN>(
                    fmt::format("Link '{}' already exists but has a different target. Deleting it", link.string()));
                fs::remove(link);
            }
        }

        if (fs::exists(link) && fs::is_symlink(link) && fs::read_symlink(link).compare(target) == 0) {
            NUClear::log<NUClear::INFO>(
                fmt::format("Link {} -> {} already exists. Skipping", target.string(), link.string()));
        }
        else {
            NUClear::log<NUClear::INFO>(fmt::format("Creating link {} -> {}", target.string(), link.string()));
            fs::create_symlink(target, link);
        }
    }

    SystemConfiguration::SystemConfiguration(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        // Check that we are running as the root user
        if (geteuid()) {
            log<NUClear::FATAL>("This command must be run as a superuser");
            exit(1);
        }

        on<Configuration>("SystemConfiguration.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            // The current user
            std::string user = config["user"].as<std::string>();

            // Get the hostname
            std::string hostname = config.hostname;

            // The base path that stores the files
            fs::path base_path = "system";

            // Our default, and platform specific paths
            fs::path default_path = base_path / "default";
            fs::path device_path  = base_path / hostname;

            /******************************
             * SYSTEM CONFIGURATION FILES *
             ******************************/
            log<NUClear::INFO>(
                fmt::format("Scanning for system files in {} and {}", default_path.string(), device_path.string()));
            for (const auto& p : fs::recursive_directory_iterator(default_path)) {
                if (p.is_regular_file()) {
                    fs::path in_file  = p.path();
                    fs::path relative = fs::relative(in_file, default_path);

                    fs::path default_file = default_path / relative;
                    fs::path device_file  = device_path / relative;
                    fs::path out_file     = "/" / relative;

                    // If we have a device specific override use that
                    if (fs::is_regular_file(device_file)) {
                        in_file = device_file;
                        log<NUClear::TRACE>(fmt::format("Using {} config for {}", hostname, out_file.string()));
                    }
                    else {
                        in_file = default_file;
                        log<NUClear::TRACE>(fmt::format("Using default config for {}", out_file.string()));
                    }

                    // If they are not the same, overwrite the old file with the new one
                    if (!files_equal(in_file, out_file)) {
                        log<NUClear::DEBUG>(fmt::format("Updating file {}", out_file.string()));
                        fs::create_directories(out_file.parent_path());
                        fs::copy_file(in_file, out_file, fs::copy_options::overwrite_existing);
                    }
                }
            }

            /***************
             * PERMISSIONS *
             ***************/
            log<NUClear::INFO>("Applying permissions changes from configuration");
            for (const auto& c : config["permissions"].config) {

                fs::path path = c.first.as<std::string>();
                log<NUClear::TRACE>(fmt::format("Checking file permissions for {}", path.string()));

                if (fs::is_regular_file(path)) {
                    // Apply permissions
                    fs::perms old_perms = fs::status(path).permissions();
                    fs::perms new_perms = static_cast<fs::perms>(c.second.as<int>());

                    if (old_perms != new_perms) {
                        log<NUClear::INFO>(fmt::format("Changing permissions on {} from {} to {}",
                                                       path.string(),
                                                       permissions_to_string(old_perms),
                                                       permissions_to_string(new_perms)));
                        fs::permissions(path, new_perms);
                    }
                }
                else {
                    log<NUClear::ERROR>(fmt::format("{} is not a regular file", path.string()));
                }
            }

            /************
             * SYMLINKS *
             ************/
            log<NUClear::INFO>("Ensuring relevant symlinks exist");
            for (const auto& l : config["links"].config) {
                std::string target = l.first.as<std::string>();
                std::string link   = l.second.as<std::string>();
                symlink(link, target);
            }

            /**********
             * PACMAN *
             **********/
            log<NUClear::INFO>("Ensuring relevant pacman packages are installed");
            log<NUClear::DEBUG>("Updating existing packages and upgrading");
            std::system("pacman -Syyuu --noconfirm --needed --overwrite \\*");
            for (const auto& c : config["pacman"].config) {
                std::string package = c.as<std::string>();
                log<NUClear::TRACE>(fmt::format("Checking package {}", package));
                if (std::system(fmt::format("pacman -Qi {} &>/dev/null", package).c_str()) != 0) {
                    log<NUClear::DEBUG>(fmt::format("Installing pacman package {}", package));
                    std::system(fmt::format("pacman -S --noconfirm --needed {}", package).c_str());
                }
            }

            /***********
             * SYSTEMD *
             ***********/
            log<NUClear::INFO>("Ensuring relevant systemd services are started");
            for (const auto& c : config["systemd"].config) {
                std::string unit = c.as<std::string>();
                log<NUClear::TRACE>(fmt::format("Checking systemd unit {}", unit));
                if (std::system(fmt::format("systemctl is-enabled --quiet {}", unit).c_str()) != 0) {
                    log<NUClear::DEBUG>(fmt::format("Enabling systemd unit {}", unit));
                    std::system(fmt::format("systemctl enable {}", unit).c_str());
                }
            }

            /**********
             * LOCALE *
             **********/
            if (config["generate_locale"].as<bool>()) {
                log<NUClear::INFO>("Ensuring locales are generated");
                std::system("locale-gen");
            }

            /********
             * GRUB *
             ********/
            if (config["generate_grub"].as<bool>()) {
                log<NUClear::INFO>("Ensuring grub config is generated");
                std::system("grub-mkconfig -o /boot/grub/grub.cfg");
            }

            /********
             * MOTD *
             ********/
            log<NUClear::INFO>("Ensuring motd is generated");
            std::ofstream ofs_motd("/etc/motd");
            std::ifstream ifs_logo("system/default/etc/motd");
            ofs_motd << ifs_logo.rdbuf() << big_text(hostname);
            ofs_motd.close();
            ifs_logo.close();

            /*********
             * HOSTS *
             *********/
            log<NUClear::INFO>("Ensuring hosts is generated");
            std::ofstream ofs_hosts("/etc/hosts");
            ofs_hosts << "127.0.0.1       localhost" << std::endl;
            ofs_hosts << "::1             localhost" << std::endl;
            ofs_hosts << "127.0.1.1       " << hostname << std::endl;
            ofs_hosts.close();

            /**********
             * GROUPS *
             **********/
            log<NUClear::INFO>("Ensuring relevant groups exist");
            std::string groups(utility::file::loadFromFile("/etc/group"));
            for (const auto& g : config["groups"].config) {
                std::string group = g.as<std::string>();
                log<NUClear::TRACE>(fmt::format("Checking group {} for user {}", group, user));

                auto pos = groups.find(group);
                if (pos == std::string::npos) {
                    log<NUClear::INFO>(
                        fmt::format("Group {} doesn't exist. Creating it and adding user {}", group, user));
                    std::system(fmt::format("groupadd {}", group).c_str());
                    std::system(fmt::format("useradd -m -G {} {}", group, user).c_str());
                }
                else {
                    auto eol = groups.find("\n", pos);
                    if (groups.substr(pos, eol - pos).find(user) == std::string::npos) {
                        log<NUClear::INFO>(fmt::format("Adding user {} to group {}", user, group));
                        std::system(fmt::format("useradd -m -G {} {}", group, user).c_str());
                    }
                }
            }

            /*******
             * ZSH *
             *******/
            log<NUClear::INFO>("Ensuring zsh is configured");
            fs::path home         = fs::path("/home") / user;
            fs::path zprezto_root = home / ".zprezto";
            if (!fs::exists(zprezto_root)) {
                log<NUClear::INFO>(fmt::format("Cloning zprezto to {}", zprezto_root.string()));
                std::system(fmt::format("git clone --recursive https://github.com/sorin-ionescu/prezto.git {}",
                                        zprezto_root.string())
                                .c_str());
            }
            else {
                log<NUClear::INFO>(fmt::format("Pulling latest zprezto in {}", zprezto_root.string()));
                std::system(fmt::format("git -C {} pull --recurse-submodules=yes", zprezto_root.string()).c_str());
            }

            log<NUClear::INFO>("Making zprezto symlinks");
            for (const auto& p : fs::directory_iterator(zprezto_root / "runcoms")) {
                if (p.is_regular_file() && p.path().filename().string()[0] == 'z') {
                    fs::path target = p.path();
                    fs::path link   = home / fmt::format(".{}", p.path().filename().string());
                    symlink(link, target);
                }
            }

            log<NUClear::INFO>("Changing user shell to zsh");
            std::system(fmt::format("chsh -s /usr/bin/zsh {}", user).c_str());

            log<NUClear::INFO>("Appending fuzzy find scripts to zshrc");
            std::ifstream ifs_zshrc(home / ".zshrc");
            std::string line;
            bool fuzzy_found = false;
            while (std::getline(ifs_zshrc, line)) {
                if (line.compare("# Source the fuzzy find scripts") == 0) {
                    fuzzy_found = true;
                    break;
                }
            }
            ifs_zshrc.close();

            if (!fuzzy_found) {
                std::ofstream ofs_zshrc(home / ".zshrc", std::ios_base::out | std::ios_base::app | std::ios_base::ate);
                ofs_zshrc << std::endl
                          << "# Source the fuzzy find scripts" << std::endl
                          << "source /usr/share/fzf/key-bindings.zsh" << std::endl
                          << "source /usr/share/fzf/completion.zsh" << std::endl;
                ofs_zshrc.close();
            }

            /**********
             * PYTHON *
             **********/
            // Make sure python checks /usr/local for packages
            std::system(
                R"X(echo $(python -c "import site; print(site.getsitepackages()[0].replace('/usr', '/usr/local'))") \
            > $(python -c "import site; print(site.getsitepackages()[0],'/local.pth',sep='')"))X");

            /***********
             * CLEANUP *
             ***********/
            log<NUClear::INFO>("Ensuring correct ownership is maintained");
            log<NUClear::INFO>(fmt::format("Forcing ownership to {0}:{0} in {1}", user, home.string()));
            std::system(fmt::format("chown -R {0}:{0} {1}", user, home.string()).c_str());
            log<NUClear::INFO>(fmt::format("Forcing ownership to {0}:{0} in /usr/local", user));
            std::system(fmt::format("chown -R {0}:{0} /usr/local", user).c_str());
        });

        // Exit here once all the reactions have run
        exit(0);
    }

}  // namespace module::tools
