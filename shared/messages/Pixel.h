/*! @file Pixel.h
    @brief Declaration of the Pixel union

 Copyright (c) 2009, 2010 Jason Kulk
 
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

#ifndef MESSAGES_PIXEL_H
#define MESSAGES_PIXEL_H

namespace messages
{

	union Pixel
	{
		unsigned color;             //!< Representation as single machine word.
		unsigned char channel[4];   //!< Representation as an array of channels.
		struct
		{
			unsigned char	yCbCrPadding,	//!< Ignore
							cb,				//!< Cb channel.
							y,				//!< Y channel.
							cr;             //!< Cr channel.
		};
	};

}
#endif
