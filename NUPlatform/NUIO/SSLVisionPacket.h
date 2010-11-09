/*! @file SSLVisionPacket.h
    @brief Declaration of SSLVisionPacket structure.

    @class SSLVisionPacket
    @brief SSLVisionPacket a network packet for ssl vision position data

    @author Steven Nicklin
 
 Copyright (c) 2010 Steven Nicklin
 
    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SSLVISIONPACKET_H
#define SSLVISIONPACKET_H

#include <vector>
#include <iostream>

struct FieldLocation
{
    float x;
    float y;
};

struct RobotLocation
{
    int id;
    FieldLocation location;
    float heading;
};

class SSLVisionPacket
{
public:
    SSLVisionPacket();
    ~SSLVisionPacket();

    friend std::ostream& operator<< (std::ostream& output, const SSLVisionPacket& p_packet);
    friend std::istream& operator>> (std::istream& input, SSLVisionPacket& p_packet);

    std::vector<RobotLocation> robots;
    std::vector<FieldLocation> balls;
};

#endif

