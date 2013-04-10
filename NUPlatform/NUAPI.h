/*! @file NUAPI.h
        @brief Declaration of nuapi class.
 
        @class NUAPI
        @brief NUAPI class for interfacing with the robot over the network

        @author Brendan Annable
 
  Copyright (c) 2010 Brendan Annable
 
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

#include <iostream>
#include <string>
#include <ctime>
#include <zmq.hpp>
#include <zhelpers.hpp>
#include <libjson/libjson.h>
#include <boost/lexical_cast.hpp>

#include <zlib.h>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include "../Infrastructure/NUImage/lib/jpge.h"

#include "../Infrastructure/NUImage/NUImage.h"
#include "../Infrastructure/NUImage/ColorModelConversions.h"
#include "../Infrastructure/NUSensorsData/NUSensorsData.h"
#include "../Infrastructure/FieldObjects/FieldObjects.h"
#include "../Infrastructure/NUBlackboard.h"
#include "../Infrastructure/NUData.h"

#include "NUAPI/proto/NUAPI.pb.h"

#ifndef NUAPI_H
#define	NUAPI_H

#define API_PORT           12000

class NUAPI {
public:	
	
    NUAPI();
    virtual ~NUAPI();

    void sendAll();
	
	void sendSensorData();
	void sendVisionData();
	void sendLocalisationData();
	
	void send(API::Message api_message);
private:
	zmq::context_t context;
	zmq::socket_t publisher;

    void send(const JSONNode& node);

	void populate_vision_field_object(std::string name, Object& field_object, API::VisionFieldObject* api_field_object, API::VisionFieldObject::Type type);
	
    template <typename T>
	void api_add_vector(API::Vector* api_vector, vector<T>& vec);
};

#endif	/* NUAPI_H */

