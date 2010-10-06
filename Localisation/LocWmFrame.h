#ifndef LOCWMFRAME_H_DEFINED
#define LOCWMFRAME_H_DEFINED
#include <iostream>
#include "Tools/FileFormats/TimestampedData.h"
#include <vector>

class Localisation;
class NUSensorsData;
class FieldObjects;
class GameInformation;
class TeamInformation;

class LocWmFrame: public TimestampedData
{
    public:
        LocWmFrame();
        LocWmFrame(Localisation* loc, NUSensorsData* sensors, FieldObjects* objects);
        ~LocWmFrame();
        double GetTimestamp() const;
    private:
        const bool m_buffered;
        Localisation* m_loc;
        NUSensorsData* m_sensors;
        FieldObjects* m_objects;

         /*!
         @brief Output streaming operation.
         @param output The output stream.
         @param p_loc The source localisation frame data to be streamed.
         */
         friend std::ostream& operator<< (std::ostream& output, const LocWmFrame& p_loc);

         /*!
         @brief Input streaming operation.
         @param input The input stream.
         @param p_kf The destination localisation frame data to be streamed to.
         */
         friend std::istream& operator>> (std::istream& input, LocWmFrame& p_loc);
};

#endif
