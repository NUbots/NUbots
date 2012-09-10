#ifndef STATIONARYOBJECT_H
#define STATIONARYOBJECT_H


#include "Object.h"
#include "../../Tools/Math/Vector2.h"
#include "../../Tools/Math/Vector3.h"
#include <string>

class StationaryObject: public Object{
	private:
                int ID;

	public:
                StationaryObject(const Vector2<float>& initialFieldLocation, int id = -1, const std::string& initName = "Unknown");
                StationaryObject(float x = 0, float y = 0, int id = -1, const std::string& initName = "Unknown");
                StationaryObject(const StationaryObject& otherObject);
		~StationaryObject();
                Vector2<float> fieldLocation;


		//Access:
                Vector2<float> getFieldLocation() const {return fieldLocation;}
		//ShortCuts:
                float X() const {return fieldLocation.x;}
                float Y() const {return fieldLocation.y;}

                std::string toString() const;

                /*!
                @brief Output streaming operation.
                @param output The output stream.
                @param p_loc The source localisation data to be streamed.
                */
                friend std::ostream& operator<< (std::ostream& output, const StationaryObject& p_stat);

                /*!
                @brief Input streaming operation.
                @param input The input stream.
                @param p_kf The destination localisation data to be streamed to.
                */
                friend std::istream& operator>> (std::istream& input, StationaryObject& p_stat);

};

#endif
