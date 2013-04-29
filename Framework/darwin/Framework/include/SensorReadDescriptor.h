
#ifndef SENSOR_READ_DESCRIPTOR_H
#define SENSOR_READ_DESCRIPTOR_H

namespace Robot
{
    //! Stores data describing how a sensor whould be read, and keeps track
    //! of how responsive the sensor is to read requests.
    class SensorReadDescriptor
    {
    public:
        SensorReadDescriptor()
        {
            sensor_id_     = 0;
            start_address_ = 0;
            num_bytes_     = 0;
            response_rate_ = 1;
        };

        int sensor_id() const { return sensor_id_; }
        int start_address() const { return start_address_; }
        int num_bytes() const { return num_bytes_; }
        double response_rate() const { return response_rate_; }
        void set_sensor_id(int sensor_id) { sensor_id_ = sensor_id; }
        void set_start_address(int address) { start_address_ = address; }
        void set_num_bytes(int num_bytes) { num_bytes_ = num_bytes; }
        void set_response_rate(double rate) { response_rate_ = rate; }

        //! Updates the response rate estimate given an error value.
        // It doesn't matter how this is done, so long as it's relatively fast
        // and results in reasonable performance.
        // returns the new response rate
        double UpdateResponseRate(int error_code);

    private:
        //! The id of the sensor to read
        int sensor_id_;
        //! The address in the sensors memory table at which to start reading
        int start_address_;
        //! The number of bytes to read from the sensor's memory table
        int num_bytes_;
        //! A value in the range [0, 1] that gives an indication of the
        //! proportion of reads from the sensor that are successful.
        double response_rate_;
    };

    class CompareSensorReadDescriptors
    {
    public:
        // Compare two SensorReadDescriptor pointers.
        bool operator()(SensorReadDescriptor*& x, 
                        SensorReadDescriptor*& y) const
        {
            return (x->response_rate() < y->response_rate());
        }
    };
}

#endif
