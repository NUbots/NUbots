#include "NUAPI.h"
#include <boost/foreach.hpp>

NUAPI::NUAPI(): context(1), publisher(context, ZMQ_PUB)
{
	//int hwm = 1;
	//publisher.setsockopt(ZMQ_SNDHWM, &hwm, sizeof (hwm));
	publisher.bind("tcp://*:12000");

}

NUAPI::~NUAPI()
{
}

void NUAPI::sendAll()
{
	//int now = clock();
	sendVisionData();
	sendSensorData();
	sendLocalisationData();
	//double diff = (clock()-now);
	//double diffms = diff/(CLOCKS_PER_SEC/1000);
	//std::cout << diffms << std::endl;

}

void NUAPI::sendVisionData()
{
	NUImage* image = Blackboard->Image;
	FieldObjects* fieldObjects = Blackboard->Objects;

	API::Message api_message;

	api_message.set_type(API::Message::VISION);
	api_message.set_utc_timestamp(std::time(0));

	API::Vision* api_vision = api_message.mutable_vision();
	API::Image* api_image = api_vision->mutable_image();

	int image_width = image->getWidth();
	int image_height = image->getHeight();

	jpge::uint8* data = new jpge::uint8[image_width * image_height * 3]();

	int index = 0;
	for (int y = image_height - 1; y >= 0; y--)
	{
		for (int x = image_width - 1; x >= 0; x--)
		{
			Pixel pixel = image->at(x, y);

			unsigned char r, g, b;
			ColorModelConversions::fromYCbCrToRGB((unsigned char) pixel.y, (unsigned char) pixel.cb, (unsigned char) pixel.cr, r, g, b);

			data[index] = r;
			data[index + 1] = g;
			data[index + 2] = b;

			index += 3;
		}
	}

	int orig_buf_size = image_width * image_height * 3; // allocate a buffer that's hopefully big enough (this is way overkill for jpeg)
	if (orig_buf_size < 1024) orig_buf_size = 1024;
	void *pBuf = malloc(orig_buf_size);
	int c_size = orig_buf_size;
	jpge::compress_image_to_jpeg_file_in_memory(pBuf, c_size, image_width, image_height, 3, data);

	delete[] data;

	api_image->set_width(image_width);
	api_image->set_height(image_height);
	api_image->set_data(pBuf, c_size);

	free(pBuf);

	MobileObject& ball = fieldObjects->mobileFieldObjects[FieldObjects::FO_BALL];

	populate_vision_field_object("ball", ball, api_vision->add_field_object(), API::VisionFieldObject::CIRCLE);
	
	
	if (Blackboard->horizontalScans != NULL && Blackboard->verticalScans != NULL)
	{
		const vector<vector<ColourSegment> >& hColourSegments = Blackboard->horizontalScans->getSegments();
		const vector<vector<ColourSegment> >& vColourSegments = Blackboard->verticalScans->getSegments();

		BOOST_FOREACH(AmbiguousObject& ambiguous_object, fieldObjects->ambiguousFieldObjects)
		{
			API::VisionFieldObject* api_field_object = api_vision->add_field_object();
			populate_vision_field_object(ambiguous_object.getName(), ambiguous_object, api_field_object, API::VisionFieldObject::RECTANGLE);
			api_field_object->set_visible(true); // hack - all ambiguous objects are visible - https://github.com/nubots/robocup/issues/10

		}

		API::VisionClassifiedImage* api_classified_image = api_vision->mutable_classified_image();

		BOOST_FOREACH(const vector<ColourSegment>& rowColourSegments, hColourSegments)
		{

			BOOST_FOREACH(const ColourSegment& colorSegment, rowColourSegments)
			{
				const Point& start = colorSegment.getStart();
				const Point& end = colorSegment.getEnd();
				Colour colour = colorSegment.getColour();

				API::VisionClassifiedSegment* api_segment = api_classified_image->add_segment();
				api_segment->set_start_x(start[0]);
				api_segment->set_start_y(start[1]);
				api_segment->set_end_x(end[0]);
				api_segment->set_end_y(end[1]);
				api_segment->set_colour(colour);
			}
		}

		BOOST_FOREACH(const vector<ColourSegment>& columnColourSegments, vColourSegments)
		{
			for (const ColourSegment& colorSegment: columnColourSegments)
			{
				const Point& start = colorSegment.getStart();
				const Point& end = colorSegment.getEnd();
				Colour colour = colorSegment.getColour();

				API::VisionClassifiedSegment* api_segment = api_classified_image->add_segment();
				api_segment->set_start_x(start[0]);
				api_segment->set_start_y(start[1]);
				api_segment->set_end_x(end[0]);
				api_segment->set_end_y(end[1]);
				api_segment->set_colour(colour);
			}
		}
	}

	send(api_message);
}

void NUAPI::sendSensorData()
{
	NUSensorsData* sensors = Blackboard->Sensors;

	API::Message api_message;

	api_message.set_type(API::Message::SENSOR_DATA);
	api_message.set_utc_timestamp(std::time(0));

	API::SensorData* api_sensor_data = api_message.mutable_sensor_data();

	BOOST_FOREACH(NUData::id_t* motor_ptr, sensors->mapIdToIds(NUSensorsData::All))
	{
		NUData::id_t& motor = *motor_ptr;

		API::Motor* api_motor = api_sensor_data->add_motor();
		api_motor->set_name(motor.Name);

		float data;
		sensors->getPosition(motor, data);
		api_motor->set_position(data);

		data = float();
		sensors->getVelocity(motor, data);
		api_motor->set_velocity(data);

		data = float();
		sensors->getAcceleration(motor, data);
		api_motor->set_acceleration(data);

		data = float();
		sensors->getTarget(motor, data);
		api_motor->set_target(data);

		data = float();
		sensors->getStiffness(motor, data);
		api_motor->set_stiffness(data);

		data = float();
		sensors->getCurrent(motor, data);
		api_motor->set_current(data);

		data = float();
		sensors->getTorque(motor, data);
		api_motor->set_torque(data);

		data = float();
		sensors->getTemperature(motor, data);
		api_motor->set_temperature(data);
	}

	vector<float> vec(3, 0);
	sensors->getAccelerometer(vec);
	API::Vector* api_accelerometer = api_sensor_data->mutable_accelerometer();
	api_add_vector(api_accelerometer, vec);

	vec = vector<float>(3, 0);
	sensors->getGyro(vec);
	API::Vector* api_gyro = api_sensor_data->mutable_gyro();
	api_add_vector(api_gyro, vec);

	vec = vector<float>(3, 0);
	sensors->getOrientation(vec);
	API::Vector* api_orientation = api_sensor_data->mutable_orientation();
	api_add_vector(api_orientation, vec);

	send(api_message);

}

void NUAPI::sendLocalisationData()
{
	FieldObjects* fieldObjects = Blackboard->Objects;
	Self& self = fieldObjects->self;
	MobileObject& ball = fieldObjects->mobileFieldObjects[FieldObjects::FO_BALL];

	API::Message api_message;

	api_message.set_type(API::Message::LOCALISATION);
	api_message.set_utc_timestamp(std::time(0));

	API::Localisation* api_localisation = api_message.mutable_localisation();

	API::LocalisationFieldObject* api_self = api_localisation->add_field_object();
	api_self->set_name("self");
	api_self->set_wm_x(self.wmX());
	api_self->set_wm_y(self.wmY());
	api_self->set_sd_x(self.sdX());
	api_self->set_sd_y(self.sdY());
	api_self->set_heading(self.Heading());
	api_self->set_sd_heading(self.sdHeading());
	api_self->set_lost(self.lost());

	API::LocalisationFieldObject* api_ball = api_localisation->add_field_object();
	api_ball->set_name("ball");
	api_ball->set_wm_x(ball.X());
	api_ball->set_wm_y(ball.Y());
	api_ball->set_sd_x(ball.sdX());
	api_ball->set_sd_y(ball.sdY());
	api_ball->set_sr_xx(ball.srXX());
	api_ball->set_sr_xy(ball.srXY());
	api_ball->set_sr_yy(ball.srYY());
	api_ball->set_lost(ball.lost());

	send(api_message);

}

void NUAPI::send(API::Message api_message)
{
	std::string output;
	api_message.SerializeToString(&output);

	s_send(publisher, output);

}

void NUAPI::populate_vision_field_object(std::string name, Object& field_object, API::VisionFieldObject* api_field_object, API::VisionFieldObject::Type type)
{
	api_field_object->set_id(field_object.getID());
	api_field_object->set_name(name);
	api_field_object->set_visible(field_object.isObjectVisible());
	api_field_object->set_screen_x(field_object.ScreenX());
	api_field_object->set_screen_y(field_object.ScreenY());
	api_field_object->set_type(type);

	switch (type)
	{
		case API::VisionFieldObject::CIRCLE:
			api_field_object->set_radius(field_object.getObjectWidth() / 2);
			break;
		case API::VisionFieldObject::RECTANGLE:
			api_field_object->set_width(field_object.getObjectWidth());
			api_field_object->set_height(field_object.getObjectHeight());
			break;
	}
}

template <typename T>
void NUAPI::api_add_vector(API::Vector* api_vector, vector<T>& vec)
{

	BOOST_FOREACH(T& value, vec)
	{
		api_vector->add_float_value(value);
	}
}