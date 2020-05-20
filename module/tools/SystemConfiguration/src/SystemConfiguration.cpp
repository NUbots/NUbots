#include "SystemConfiguration.h"

#include <fmt/format.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "extension/Configuration.h"

namespace module {
namespace tools {

    using extension::Configuration;
    namespace fs = std::filesystem;

    bool files_equal(const fs::path& a, const fs::path& b) {
        // If both files exist and are the same size
        if (fs::is_regular_file(a) && fs::is_regular_file(b) && fs::file_size(a) == fs::file_size(b)) {
            std::ifstream a_f(a);
            std::ifstream b_f(b);
            return std::equal(std::istreambuf_iterator<char>(a_f),
                              std::istreambuf_iterator<char>(),
                              std::istreambuf_iterator<char>(b_f));
        }
        else {
            return false;
        }
    }

    std::string permissions_to_string(const fs::perms& p) {
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

    SystemConfiguration::SystemConfiguration(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        // Check that we are running as the root user
        if (geteuid()) {
            log<NUClear::FATAL>("This command must be run as a superuser");
            exit(1);
        }

        on<Configuration>("SystemConfiguration.yaml").then([this](const Configuration& config) {
            if (config["log_level"].as<std::string>() == "TRACE") log_level = NUClear::TRACE;
            if (config["log_level"].as<std::string>() == "DEBUG") log_level = NUClear::DEBUG;
            if (config["log_level"].as<std::string>() == "INFO") log_level = NUClear::INFO;
            if (config["log_level"].as<std::string>() == "WARN") log_level = NUClear::WARN;
            if (config["log_level"].as<std::string>() == "ERROR") log_level = NUClear::ERROR;
            if (config["log_level"].as<std::string>() == "FATAL") log_level = NUClear::FATAL;

            // Get the hostname
            char hostname_c[255];
            if (gethostname(hostname_c, 255) != 0) {
                throw std::system_error(errno, std::system_category(), "Error getting hostname.");
            }
            std::string hostname = hostname_c;

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
                log<NUClear::TRACE>(fmt::format("Checking link {} -> {}", link, target));
                if (fs::exists(target)) {
                    log<NUClear::WARN>(fmt::format("Link target '{}' doesn't exist. Skipping", target));
                }
                else if (!fs::exists(link) || fs::is_symlink(link)) {
                    if (fs::exists(link) && fs::read_symlink(link).compare(target) == 0) {
                        log<NUClear::DEBUG>(fmt::format("Link '{} -> {}' already exists", link, target));
                    }
                    else {
                        log<NUClear::DEBUG>(fmt::format("Creating link {} -> {}", target, link));
                        fs::create_symlink(target, link);
                    }
                }
                else {
                    log<NUClear::WARN>(fmt::format("{} is not a symlink. Skipping", link));
                }
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
        });

        // Exit here once all the reactions have run
        exit(0);
    }

}  // namespace tools
}  // namespace module
