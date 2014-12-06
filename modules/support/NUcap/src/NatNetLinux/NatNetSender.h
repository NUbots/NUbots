/*
 * NatNetSender.h is part of NatNetLinux, and is Copyright 2013-2014,
 * Philip G. Lee <rocketman768@gmail.com>
 *
 * NatNetLinux is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NatNetLinux is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NatNetLinux.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef NATNETSENDER_H
#define NATNETSENDER_H

#include <string>

/*!
 * \brief Encapsulates NatNet Sender packet data.
 * \author Philip G. Lee
 *
 * NatNet requires that servers respond with some basic information about
 * themselves. This class encapsulates that information.
 */
class NatNetSender {
private:
    static const unsigned int MAX_NAMELENGTH = 256;
public:
    //! \brief Default constructor
    NatNetSender();

    //! \brief Copy constructor
    NatNetSender(const NatNetSender& other);

    ~NatNetSender();

    //! \brief Assignment operator
    NatNetSender& operator=(const NatNetSender& other);

    //! \brief Name of sending application.
    std::string name() const;

    //! \brief Length 4 array version number of sending application (major.minor.build.revision)
    const unsigned char* version() const;

    //! \brief Length 4 array version number of sending application's NatNet version (major.minor.build.revision)
    const unsigned char* natNetVersion() const;

    //! \brief Unpack the class from raw pointer.
    void unpack(const char* data);

private:

    char _name[MAX_NAMELENGTH];
    unsigned char _version[4];
    unsigned char _natNetVersion[4];
};

#endif /*NATNETSENDER_H*/
