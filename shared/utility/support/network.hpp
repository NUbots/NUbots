/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
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
#ifndef UTILITY_SUPPORT_NETWORK_HPP
#define UTILITY_SUPPORT_NETWORK_HPP

#include <fmt/format.h>
#include <string>

extern "C" {
#include <arpa/inet.h>
#include <cstring>
#include <ifaddrs.h>
#include <linux/wireless.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <unistd.h>
}

namespace utility::support {

    inline std::string get_hostname() {
        char* hostname = std::getenv("ROBOT_HOSTNAME");

        if (hostname != nullptr) {
            return std::string(hostname);
        }

        char buffer[255];
        ::gethostname(buffer, 255);
        return std::string(buffer);
    }

    inline std::string get_ip_address(const std::string& interface_name) {
        struct ifaddrs *ifaddr, *ifa;
        char ip[NI_MAXHOST];

        if (getifaddrs(&ifaddr) == -1) {
            return "";
        }

        for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
            if (ifa->ifa_addr == NULL)
                continue;

            if (ifa->ifa_addr->sa_family == AF_INET) {  // check it is IP4
                // is a valid IP4 Address
                void* tmpAddrPtr = &((struct sockaddr_in*) ifa->ifa_addr)->sin_addr;
                inet_ntop(AF_INET, tmpAddrPtr, ip, NI_MAXHOST);
                if (strcmp(ifa->ifa_name, interface_name.c_str()) == 0) {
                    freeifaddrs(ifaddr);
                    return ip;
                }
            }
        }

        freeifaddrs(ifaddr);
        return "";
    }

    inline std::string get_wireless_interface() {
        struct ifaddrs *ifaddr, *ifa;
        int sock = socket(AF_INET, SOCK_DGRAM, 0);

        if (getifaddrs(&ifaddr) == -1) {
            close(sock);
            return "";
        }

        for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
            if (ifa->ifa_addr == NULL)
                continue;

            struct iwreq pwrq;
            memset(&pwrq, 0, sizeof(pwrq));
            strncpy(pwrq.ifr_name, ifa->ifa_name, IFNAMSIZ);

            if (ioctl(sock, SIOCGIWNAME, &pwrq) != -1) {
                freeifaddrs(ifaddr);
                close(sock);
                return ifa->ifa_name;
            }
        }

        freeifaddrs(ifaddr);
        close(sock);
        return "";
    }

    inline std::string get_ssid(const std::string& interface_name) {
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        struct iwreq pwrq;
        memset(&pwrq, 0, sizeof(pwrq));
        strncpy(pwrq.ifr_name, interface_name.c_str(), IFNAMSIZ);

        char ssid[IW_ESSID_MAX_SIZE + 1];
        pwrq.u.essid.pointer = ssid;
        pwrq.u.essid.length  = IW_ESSID_MAX_SIZE;

        if (ioctl(sock, SIOCGIWESSID, &pwrq) == -1) {
            close(sock);
            return "";
        }

        close(sock);
        ssid[pwrq.u.essid.length] = '\0';
        return std::string(ssid);
    }

    inline std::string get_wifi_password(const std::string& ssid, const std::string& wifi_interface) {
        std::string wpa_supplicant_file = fmt::format("/etc/wpa_supplicant/wpa_supplicant-{}.conf", wifi_interface);
        std::ifstream file(wpa_supplicant_file);
        std::string line;
        bool ssid_found = false;
        char c;
        std::string buffer;

        while (file.get(c)) {
            buffer.push_back(c);

            if (buffer.size() > 4) {
                buffer.erase(buffer.begin());
            }

            if (ssid_found && buffer == "psk=") {
                std::string psk;
                // Skip whitespace and the opening quote
                while (file.get(c) && isspace(c))
                    ;
                if (c != '\"')
                    return "";  // Expected a quote

                // Read the psk value inside the quotes
                while (file.get(c) && c != '\"') {
                    psk.push_back(c);
                }

                return psk;
            }

            if (!ssid_found && buffer == "ssid") {
                std::string current_ssid;
                // Skip whitespace and the equals sign
                while (file.get(c) && (isspace(c) || c == '='))
                    ;
                if (c != '\"')
                    return "";  // Expected a quote

                // Read the ssid value inside the quotes
                while (file.get(c) && c != '\"') {
                    current_ssid.push_back(c);
                }

                if (current_ssid == ssid) {
                    ssid_found = true;
                }
            }
        }

        // If we reach here, we didn't find the ssid or the psk
        return "";
    }

}  // namespace utility::support

#endif  // UTILITY_SUPPORT_NETWORK_HPP
