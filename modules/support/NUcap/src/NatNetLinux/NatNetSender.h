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

#include <string.h>
#include <string>

/*!
 * \brief Encapsulates NatNet Sender packet data.
 * \author Philip G. Lee
 * 
 * NatNet requires that servers respond with some basic information about
 * themselves. This class encapsulates that information.
 */
class NatNetSender
{
#define MAX_NAMELENGTH 256
public:
   //! \brief Default constructor
   NatNetSender()
   {
      memset(_name, 0, MAX_NAMELENGTH);
      memset(_version, 0, 4);
      memset(_natNetVersion, 0, 4);
   }
   
   //! \brief Copy constructor
   NatNetSender( NatNetSender const& other )
   {
      memcpy( _name, other._name, MAX_NAMELENGTH );
      memcpy( _version, other._version, 4 );
      memcpy( _natNetVersion, other._natNetVersion, 4 );
   }
   
   ~NatNetSender(){}
   
   //! \brief Assignment operator
   NatNetSender& operator=( NatNetSender const& other )
   {
      memmove( _name, other._name, MAX_NAMELENGTH );
      memmove( _version, other._version, 4 );
      memmove( _natNetVersion, other._natNetVersion, 4 );
      return *this;
   }
   
   //! \brief Name of sending application.
   std::string name() const
   {
      return _name;
   }
   
   //! \brief Length 4 array version number of sending application (major.minor.build.revision)
   unsigned char const* version() const
   {
      return _version;
   }
   
   //! \brief Length 4 array version number of sending application's NatNet version (major.minor.build.revision)
   unsigned char const* natNetVersion() const
   {
      return _natNetVersion;
   }
   
   //! \brief Unpack the class from raw pointer.
   void unpack(char const* data)
   {
      // NOTE: do we have to worry about network order data? I.e. ntohs() and stuff?
      strncpy( _name, data, MAX_NAMELENGTH );
      data += MAX_NAMELENGTH;
      _version[0]       = data[0];
      _version[1]       = data[1];
      _version[2]       = data[2];
      _version[3]       = data[3];
      _natNetVersion[0] = data[4];
      _natNetVersion[1] = data[5];
      _natNetVersion[2] = data[6];
      _natNetVersion[3] = data[7];
   }
   
private:
   
   char _name[MAX_NAMELENGTH];
   unsigned char _version[4];
   unsigned char _natNetVersion[4];
   
#undef MAX_NAMELENGTH
};

#endif /*NATNETSENDER_H*/
