#ifndef MESSAGE_PLATFORM_WEBOTS_MESSAGES_HPP
#define MESSAGE_PLATFORM_WEBOTS_MESSAGES_HPP

#include <cstdint>
#include <array>
#include <exception>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "message/platform/webots/messages.pb.h"
#include "message/MessageBase.hpp"

namespace message::platform::webots {

    // Enum Definitions

    // Message Definitions
    struct Vector3 : public ::message::MessageBase<Vector3> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::Vector3;
    
        // Enum Definitions
    
        // Submessage Definitions
    
        // Constructors
        Vector3(double const& X = 0.0, double const& Y = 0.0, double const& Z = 0.0);
        
        
        Vector3(const Vector3&) = default;
        Vector3(Vector3&&) = default;
        ~Vector3() = default;
        Vector3& operator=(const Vector3&) = default;
        Vector3& operator=(Vector3&&) = default;
        
        Vector3(const ::protobuf::message::platform::webots::Vector3& proto);
        
        bool operator== (const Vector3& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::Vector3() const;
        // Fields
        double X;
        double Y;
        double Z;
    };
    
    struct PositionSensorMeasurement : public ::message::MessageBase<PositionSensorMeasurement> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::PositionSensorMeasurement;
    
        // Enum Definitions
    
        // Submessage Definitions
    
        // Constructors
        PositionSensorMeasurement(::std::string name = "", double const& value = 0.0);
        
        
        PositionSensorMeasurement(const PositionSensorMeasurement&) = default;
        PositionSensorMeasurement(PositionSensorMeasurement&&) = default;
        ~PositionSensorMeasurement() = default;
        PositionSensorMeasurement& operator=(const PositionSensorMeasurement&) = default;
        PositionSensorMeasurement& operator=(PositionSensorMeasurement&&) = default;
        
        PositionSensorMeasurement(const ::protobuf::message::platform::webots::PositionSensorMeasurement& proto);
        
        bool operator== (const PositionSensorMeasurement& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::PositionSensorMeasurement() const;
        // Fields
        ::std::string name;
        double value;
    };
    
    struct AccelerometerMeasurement : public ::message::MessageBase<AccelerometerMeasurement> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::AccelerometerMeasurement;
    
        // Enum Definitions
    
        // Submessage Definitions
    
        // Constructors
        AccelerometerMeasurement(::std::string name = "", ::message::platform::webots::Vector3 value = ::message::platform::webots::Vector3());
        
        
        AccelerometerMeasurement(const AccelerometerMeasurement&) = default;
        AccelerometerMeasurement(AccelerometerMeasurement&&) = default;
        ~AccelerometerMeasurement() = default;
        AccelerometerMeasurement& operator=(const AccelerometerMeasurement&) = default;
        AccelerometerMeasurement& operator=(AccelerometerMeasurement&&) = default;
        
        AccelerometerMeasurement(const ::protobuf::message::platform::webots::AccelerometerMeasurement& proto);
        
        bool operator== (const AccelerometerMeasurement& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::AccelerometerMeasurement() const;
        // Fields
        ::std::string name;
        ::message::platform::webots::Vector3 value;
    };
    
    struct GyroMeasurement : public ::message::MessageBase<GyroMeasurement> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::GyroMeasurement;
    
        // Enum Definitions
    
        // Submessage Definitions
    
        // Constructors
        GyroMeasurement(::std::string name = "", ::message::platform::webots::Vector3 value = ::message::platform::webots::Vector3());
        
        
        GyroMeasurement(const GyroMeasurement&) = default;
        GyroMeasurement(GyroMeasurement&&) = default;
        ~GyroMeasurement() = default;
        GyroMeasurement& operator=(const GyroMeasurement&) = default;
        GyroMeasurement& operator=(GyroMeasurement&&) = default;
        
        GyroMeasurement(const ::protobuf::message::platform::webots::GyroMeasurement& proto);
        
        bool operator== (const GyroMeasurement& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::GyroMeasurement() const;
        // Fields
        ::std::string name;
        ::message::platform::webots::Vector3 value;
    };
    
    struct BumperMeasurement : public ::message::MessageBase<BumperMeasurement> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::BumperMeasurement;
    
        // Enum Definitions
    
        // Submessage Definitions
    
        // Constructors
        BumperMeasurement(::std::string name = "", bool const& value = false);
        
        
        BumperMeasurement(const BumperMeasurement&) = default;
        BumperMeasurement(BumperMeasurement&&) = default;
        ~BumperMeasurement() = default;
        BumperMeasurement& operator=(const BumperMeasurement&) = default;
        BumperMeasurement& operator=(BumperMeasurement&&) = default;
        
        BumperMeasurement(const ::protobuf::message::platform::webots::BumperMeasurement& proto);
        
        bool operator== (const BumperMeasurement& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::BumperMeasurement() const;
        // Fields
        ::std::string name;
        bool value;
    };
    
    struct ForceMeasurement : public ::message::MessageBase<ForceMeasurement> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::ForceMeasurement;
    
        // Enum Definitions
    
        // Submessage Definitions
    
        // Constructors
        ForceMeasurement(::std::string name = "", double const& value = 0.0);
        
        
        ForceMeasurement(const ForceMeasurement&) = default;
        ForceMeasurement(ForceMeasurement&&) = default;
        ~ForceMeasurement() = default;
        ForceMeasurement& operator=(const ForceMeasurement&) = default;
        ForceMeasurement& operator=(ForceMeasurement&&) = default;
        
        ForceMeasurement(const ::protobuf::message::platform::webots::ForceMeasurement& proto);
        
        bool operator== (const ForceMeasurement& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::ForceMeasurement() const;
        // Fields
        ::std::string name;
        double value;
    };
    
    struct Force3DMeasurement : public ::message::MessageBase<Force3DMeasurement> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::Force3DMeasurement;
    
        // Enum Definitions
    
        // Submessage Definitions
    
        // Constructors
        Force3DMeasurement(::std::string name = "", ::message::platform::webots::Vector3 value = ::message::platform::webots::Vector3());
        
        
        Force3DMeasurement(const Force3DMeasurement&) = default;
        Force3DMeasurement(Force3DMeasurement&&) = default;
        ~Force3DMeasurement() = default;
        Force3DMeasurement& operator=(const Force3DMeasurement&) = default;
        Force3DMeasurement& operator=(Force3DMeasurement&&) = default;
        
        Force3DMeasurement(const ::protobuf::message::platform::webots::Force3DMeasurement& proto);
        
        bool operator== (const Force3DMeasurement& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::Force3DMeasurement() const;
        // Fields
        ::std::string name;
        ::message::platform::webots::Vector3 value;
    };
    
    struct Force6DMeasurement : public ::message::MessageBase<Force6DMeasurement> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::Force6DMeasurement;
    
        // Enum Definitions
    
        // Submessage Definitions
    
        // Constructors
        Force6DMeasurement(::std::string name = "", ::message::platform::webots::Vector3 force = ::message::platform::webots::Vector3(), ::message::platform::webots::Vector3 torque = ::message::platform::webots::Vector3());
        
        
        Force6DMeasurement(const Force6DMeasurement&) = default;
        Force6DMeasurement(Force6DMeasurement&&) = default;
        ~Force6DMeasurement() = default;
        Force6DMeasurement& operator=(const Force6DMeasurement&) = default;
        Force6DMeasurement& operator=(Force6DMeasurement&&) = default;
        
        Force6DMeasurement(const ::protobuf::message::platform::webots::Force6DMeasurement& proto);
        
        bool operator== (const Force6DMeasurement& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::Force6DMeasurement() const;
        // Fields
        ::std::string name;
        ::message::platform::webots::Vector3 force;
        ::message::platform::webots::Vector3 torque;
    };
    
    struct CameraMeasurement : public ::message::MessageBase<CameraMeasurement> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::CameraMeasurement;
    
        // Enum Definitions
    
        // Submessage Definitions
    
        // Constructors
        CameraMeasurement(::std::string name = "", uint32_t const& width = 0, uint32_t const& height = 0, int32_t const& quality = 0, ::std::vector<uint8_t> image = ::std::vector<uint8_t>());
        
        
        CameraMeasurement(const CameraMeasurement&) = default;
        CameraMeasurement(CameraMeasurement&&) = default;
        ~CameraMeasurement() = default;
        CameraMeasurement& operator=(const CameraMeasurement&) = default;
        CameraMeasurement& operator=(CameraMeasurement&&) = default;
        
        CameraMeasurement(const ::protobuf::message::platform::webots::CameraMeasurement& proto);
        
        bool operator== (const CameraMeasurement& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::CameraMeasurement() const;
        // Fields
        ::std::string name;
        uint32_t width;
        uint32_t height;
        int32_t quality;
        ::std::vector<uint8_t> image;
    };
    
    struct Message : public ::message::MessageBase<Message> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::Message;
    
        // Enum Definitions
        struct MessageType : public ::message::MessageBase<MessageType> {
            enum Value {
                ERROR_MESSAGE = 0,
                WARNING_MESSAGE = 1
            };
            Value value{Value::ERROR_MESSAGE};
        
            // Constructors
            MessageType();
        
            MessageType(int const& v);
        
            MessageType(Value const& value);
        
            MessageType(std::string const& str);
        
            MessageType(::protobuf::message::platform::webots::Message::MessageType const& p);
        
            // Operators
            bool operator <(MessageType const& other) const;
        
            bool operator >(MessageType const& other) const;
        
            bool operator <=(MessageType const& other) const;
        
            bool operator >=(MessageType const& other) const;
        
            bool operator ==(MessageType const& other) const;
        
            bool operator !=(MessageType const& other) const;
        
            bool operator <(MessageType::Value const& other) const;
        
            bool operator >(MessageType::Value const& other) const;
        
            bool operator <=(MessageType::Value const& other) const;
        
            bool operator >=(MessageType::Value const& other) const;
        
            bool operator ==(MessageType::Value const& other) const;
        
            bool operator !=(MessageType::Value const& other) const;
        
            // Conversions
            operator Value() const;
        
            operator int() const;
        
            operator std::string() const;
        
            operator ::protobuf::message::platform::webots::Message::MessageType() const;
        
            friend std::ostream& operator<< (std::ostream& out, const MessageType& val);
        };
        // Submessage Definitions
    
        // Constructors
        Message(::message::platform::webots::Message::MessageType message_type = ::message::platform::webots::Message::MessageType(), ::std::string text = "");
        
        
        Message(const Message&) = default;
        Message(Message&&) = default;
        ~Message() = default;
        Message& operator=(const Message&) = default;
        Message& operator=(Message&&) = default;
        
        Message(const ::protobuf::message::platform::webots::Message& proto);
        
        bool operator== (const Message& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::Message() const;
        // Fields
        ::message::platform::webots::Message::MessageType message_type;
        ::std::string text;
    };
    
    struct OptimisationRobotPosition : public ::message::MessageBase<OptimisationRobotPosition> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::OptimisationRobotPosition;
    
        // Enum Definitions
    
        // Submessage Definitions
    
        // Constructors
        OptimisationRobotPosition(::message::platform::webots::Vector3 value = ::message::platform::webots::Vector3());
        
        
        OptimisationRobotPosition(const OptimisationRobotPosition&) = default;
        OptimisationRobotPosition(OptimisationRobotPosition&&) = default;
        ~OptimisationRobotPosition() = default;
        OptimisationRobotPosition& operator=(const OptimisationRobotPosition&) = default;
        OptimisationRobotPosition& operator=(OptimisationRobotPosition&&) = default;
        
        OptimisationRobotPosition(const ::protobuf::message::platform::webots::OptimisationRobotPosition& proto);
        
        bool operator== (const OptimisationRobotPosition& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::OptimisationRobotPosition() const;
        // Fields
        ::message::platform::webots::Vector3 value;
    };
    
    struct SensorMeasurements : public ::message::MessageBase<SensorMeasurements> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::SensorMeasurements;
    
        // Enum Definitions
    
        // Submessage Definitions
    
        // Constructors
        SensorMeasurements(uint32_t const& time = 0, uint64_t const& real_time = 0, ::std::vector<::message::platform::webots::Message> messages = ::std::vector<::message::platform::webots::Message>(), ::std::vector<::message::platform::webots::AccelerometerMeasurement> accelerometers = ::std::vector<::message::platform::webots::AccelerometerMeasurement>(), ::std::vector<::message::platform::webots::BumperMeasurement> bumpers = ::std::vector<::message::platform::webots::BumperMeasurement>(), ::std::vector<::message::platform::webots::CameraMeasurement> cameras = ::std::vector<::message::platform::webots::CameraMeasurement>(), ::std::vector<::message::platform::webots::ForceMeasurement> forces = ::std::vector<::message::platform::webots::ForceMeasurement>(), ::std::vector<::message::platform::webots::Force3DMeasurement> force3ds = ::std::vector<::message::platform::webots::Force3DMeasurement>(), ::std::vector<::message::platform::webots::Force6DMeasurement> force6ds = ::std::vector<::message::platform::webots::Force6DMeasurement>(), ::std::vector<::message::platform::webots::GyroMeasurement> gyros = ::std::vector<::message::platform::webots::GyroMeasurement>(), ::std::vector<::message::platform::webots::PositionSensorMeasurement> position_sensors = ::std::vector<::message::platform::webots::PositionSensorMeasurement>(), ::message::platform::webots::OptimisationRobotPosition robot_position = ::message::platform::webots::OptimisationRobotPosition());
        
        
        SensorMeasurements(const SensorMeasurements&) = default;
        SensorMeasurements(SensorMeasurements&&) = default;
        ~SensorMeasurements() = default;
        SensorMeasurements& operator=(const SensorMeasurements&) = default;
        SensorMeasurements& operator=(SensorMeasurements&&) = default;
        
        SensorMeasurements(const ::protobuf::message::platform::webots::SensorMeasurements& proto);
        
        bool operator== (const SensorMeasurements& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::SensorMeasurements() const;
        // Fields
        uint32_t time;
        uint64_t real_time;
        ::std::vector<::message::platform::webots::Message> messages;
        ::std::vector<::message::platform::webots::AccelerometerMeasurement> accelerometers;
        ::std::vector<::message::platform::webots::BumperMeasurement> bumpers;
        ::std::vector<::message::platform::webots::CameraMeasurement> cameras;
        ::std::vector<::message::platform::webots::ForceMeasurement> forces;
        ::std::vector<::message::platform::webots::Force3DMeasurement> force3ds;
        ::std::vector<::message::platform::webots::Force6DMeasurement> force6ds;
        ::std::vector<::message::platform::webots::GyroMeasurement> gyros;
        ::std::vector<::message::platform::webots::PositionSensorMeasurement> position_sensors;
        ::message::platform::webots::OptimisationRobotPosition robot_position;
    };
    
    struct MotorPosition : public ::message::MessageBase<MotorPosition> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::MotorPosition;
    
        // Enum Definitions
    
        // Submessage Definitions
    
        // Constructors
        MotorPosition(::std::string name = "", double const& position = 0.0);
        
        
        MotorPosition(const MotorPosition&) = default;
        MotorPosition(MotorPosition&&) = default;
        ~MotorPosition() = default;
        MotorPosition& operator=(const MotorPosition&) = default;
        MotorPosition& operator=(MotorPosition&&) = default;
        
        MotorPosition(const ::protobuf::message::platform::webots::MotorPosition& proto);
        
        bool operator== (const MotorPosition& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::MotorPosition() const;
        // Fields
        ::std::string name;
        double position;
    };
    
    struct MotorVelocity : public ::message::MessageBase<MotorVelocity> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::MotorVelocity;
    
        // Enum Definitions
    
        // Submessage Definitions
    
        // Constructors
        MotorVelocity(::std::string name = "", double const& velocity = 0.0);
        
        
        MotorVelocity(const MotorVelocity&) = default;
        MotorVelocity(MotorVelocity&&) = default;
        ~MotorVelocity() = default;
        MotorVelocity& operator=(const MotorVelocity&) = default;
        MotorVelocity& operator=(MotorVelocity&&) = default;
        
        MotorVelocity(const ::protobuf::message::platform::webots::MotorVelocity& proto);
        
        bool operator== (const MotorVelocity& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::MotorVelocity() const;
        // Fields
        ::std::string name;
        double velocity;
    };
    
    struct MotorForce : public ::message::MessageBase<MotorForce> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::MotorForce;
    
        // Enum Definitions
    
        // Submessage Definitions
    
        // Constructors
        MotorForce(::std::string name = "", double const& force = 0.0);
        
        
        MotorForce(const MotorForce&) = default;
        MotorForce(MotorForce&&) = default;
        ~MotorForce() = default;
        MotorForce& operator=(const MotorForce&) = default;
        MotorForce& operator=(MotorForce&&) = default;
        
        MotorForce(const ::protobuf::message::platform::webots::MotorForce& proto);
        
        bool operator== (const MotorForce& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::MotorForce() const;
        // Fields
        ::std::string name;
        double force;
    };
    
    struct MotorTorque : public ::message::MessageBase<MotorTorque> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::MotorTorque;
    
        // Enum Definitions
    
        // Submessage Definitions
    
        // Constructors
        MotorTorque(::std::string name = "", double const& torque = 0.0);
        
        
        MotorTorque(const MotorTorque&) = default;
        MotorTorque(MotorTorque&&) = default;
        ~MotorTorque() = default;
        MotorTorque& operator=(const MotorTorque&) = default;
        MotorTorque& operator=(MotorTorque&&) = default;
        
        MotorTorque(const ::protobuf::message::platform::webots::MotorTorque& proto);
        
        bool operator== (const MotorTorque& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::MotorTorque() const;
        // Fields
        ::std::string name;
        double torque;
    };
    
    struct MotorPID : public ::message::MessageBase<MotorPID> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::MotorPID;
    
        // Enum Definitions
    
        // Submessage Definitions
    
        // Constructors
        MotorPID(::std::string name = "", ::message::platform::webots::Vector3 PID = ::message::platform::webots::Vector3());
        
        
        MotorPID(const MotorPID&) = default;
        MotorPID(MotorPID&&) = default;
        ~MotorPID() = default;
        MotorPID& operator=(const MotorPID&) = default;
        MotorPID& operator=(MotorPID&&) = default;
        
        MotorPID(const ::protobuf::message::platform::webots::MotorPID& proto);
        
        bool operator== (const MotorPID& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::MotorPID() const;
        // Fields
        ::std::string name;
        ::message::platform::webots::Vector3 PID;
    };
    
    struct SensorTimeStep : public ::message::MessageBase<SensorTimeStep> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::SensorTimeStep;
    
        // Enum Definitions
    
        // Submessage Definitions
    
        // Constructors
        SensorTimeStep(::std::string name = "", uint32_t const& timeStep = 0);
        
        
        SensorTimeStep(const SensorTimeStep&) = default;
        SensorTimeStep(SensorTimeStep&&) = default;
        ~SensorTimeStep() = default;
        SensorTimeStep& operator=(const SensorTimeStep&) = default;
        SensorTimeStep& operator=(SensorTimeStep&&) = default;
        
        SensorTimeStep(const ::protobuf::message::platform::webots::SensorTimeStep& proto);
        
        bool operator== (const SensorTimeStep& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::SensorTimeStep() const;
        // Fields
        ::std::string name;
        uint32_t timeStep;
    };
    
    struct CameraQuality : public ::message::MessageBase<CameraQuality> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::CameraQuality;
    
        // Enum Definitions
    
        // Submessage Definitions
    
        // Constructors
        CameraQuality(::std::string name = "", int32_t const& quality = 0);
        
        
        CameraQuality(const CameraQuality&) = default;
        CameraQuality(CameraQuality&&) = default;
        ~CameraQuality() = default;
        CameraQuality& operator=(const CameraQuality&) = default;
        CameraQuality& operator=(CameraQuality&&) = default;
        
        CameraQuality(const ::protobuf::message::platform::webots::CameraQuality& proto);
        
        bool operator== (const CameraQuality& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::CameraQuality() const;
        // Fields
        ::std::string name;
        int32_t quality;
    };
    
    struct CameraExposure : public ::message::MessageBase<CameraExposure> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::CameraExposure;
    
        // Enum Definitions
    
        // Submessage Definitions
    
        // Constructors
        CameraExposure(::std::string name = "", double const& exposure = 0.0);
        
        
        CameraExposure(const CameraExposure&) = default;
        CameraExposure(CameraExposure&&) = default;
        ~CameraExposure() = default;
        CameraExposure& operator=(const CameraExposure&) = default;
        CameraExposure& operator=(CameraExposure&&) = default;
        
        CameraExposure(const ::protobuf::message::platform::webots::CameraExposure& proto);
        
        bool operator== (const CameraExposure& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::CameraExposure() const;
        // Fields
        ::std::string name;
        double exposure;
    };
    
    struct OptimisationCommand : public ::message::MessageBase<OptimisationCommand> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::OptimisationCommand;
    
        // Enum Definitions
        struct CommandType : public ::message::MessageBase<CommandType> {
            enum Value {
                RESET_WORLD = 0,
                RESET_TIME = 1,
                TERMINATE = 2
            };
            Value value{Value::RESET_WORLD};
        
            // Constructors
            CommandType();
        
            CommandType(int const& v);
        
            CommandType(Value const& value);
        
            CommandType(std::string const& str);
        
            CommandType(::protobuf::message::platform::webots::OptimisationCommand::CommandType const& p);
        
            // Operators
            bool operator <(CommandType const& other) const;
        
            bool operator >(CommandType const& other) const;
        
            bool operator <=(CommandType const& other) const;
        
            bool operator >=(CommandType const& other) const;
        
            bool operator ==(CommandType const& other) const;
        
            bool operator !=(CommandType const& other) const;
        
            bool operator <(CommandType::Value const& other) const;
        
            bool operator >(CommandType::Value const& other) const;
        
            bool operator <=(CommandType::Value const& other) const;
        
            bool operator >=(CommandType::Value const& other) const;
        
            bool operator ==(CommandType::Value const& other) const;
        
            bool operator !=(CommandType::Value const& other) const;
        
            // Conversions
            operator Value() const;
        
            operator int() const;
        
            operator std::string() const;
        
            operator ::protobuf::message::platform::webots::OptimisationCommand::CommandType() const;
        
            friend std::ostream& operator<< (std::ostream& out, const CommandType& val);
        };
        // Submessage Definitions
    
        // Constructors
        OptimisationCommand(::message::platform::webots::OptimisationCommand::CommandType command = ::message::platform::webots::OptimisationCommand::CommandType());
        
        
        OptimisationCommand(const OptimisationCommand&) = default;
        OptimisationCommand(OptimisationCommand&&) = default;
        ~OptimisationCommand() = default;
        OptimisationCommand& operator=(const OptimisationCommand&) = default;
        OptimisationCommand& operator=(OptimisationCommand&&) = default;
        
        OptimisationCommand(const ::protobuf::message::platform::webots::OptimisationCommand& proto);
        
        bool operator== (const OptimisationCommand& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::OptimisationCommand() const;
        // Fields
        ::message::platform::webots::OptimisationCommand::CommandType command;
    };
    
    struct ActuatorRequests : public ::message::MessageBase<ActuatorRequests> {
        // Protobuf type
        using protobuf_type = ::protobuf::message::platform::webots::ActuatorRequests;
    
        // Enum Definitions
    
        // Submessage Definitions
    
        // Constructors
        ActuatorRequests(::std::vector<::message::platform::webots::MotorPosition> motor_positions = ::std::vector<::message::platform::webots::MotorPosition>(), ::std::vector<::message::platform::webots::MotorVelocity> motor_velocities = ::std::vector<::message::platform::webots::MotorVelocity>(), ::std::vector<::message::platform::webots::MotorForce> motor_forces = ::std::vector<::message::platform::webots::MotorForce>(), ::std::vector<::message::platform::webots::MotorTorque> motor_torques = ::std::vector<::message::platform::webots::MotorTorque>(), ::std::vector<::message::platform::webots::MotorPID> motor_pids = ::std::vector<::message::platform::webots::MotorPID>(), ::std::vector<::message::platform::webots::SensorTimeStep> sensor_time_steps = ::std::vector<::message::platform::webots::SensorTimeStep>(), ::std::vector<::message::platform::webots::CameraQuality> camera_qualities = ::std::vector<::message::platform::webots::CameraQuality>(), ::std::vector<::message::platform::webots::CameraExposure> camera_exposures = ::std::vector<::message::platform::webots::CameraExposure>(), ::message::platform::webots::OptimisationCommand optimisation_command = ::message::platform::webots::OptimisationCommand());
        
        
        ActuatorRequests(const ActuatorRequests&) = default;
        ActuatorRequests(ActuatorRequests&&) = default;
        ~ActuatorRequests() = default;
        ActuatorRequests& operator=(const ActuatorRequests&) = default;
        ActuatorRequests& operator=(ActuatorRequests&&) = default;
        
        ActuatorRequests(const ::protobuf::message::platform::webots::ActuatorRequests& proto);
        
        bool operator== (const ActuatorRequests& other) const;
        // Converters
        operator ::protobuf::message::platform::webots::ActuatorRequests() const;
        // Fields
        ::std::vector<::message::platform::webots::MotorPosition> motor_positions;
        ::std::vector<::message::platform::webots::MotorVelocity> motor_velocities;
        ::std::vector<::message::platform::webots::MotorForce> motor_forces;
        ::std::vector<::message::platform::webots::MotorTorque> motor_torques;
        ::std::vector<::message::platform::webots::MotorPID> motor_pids;
        ::std::vector<::message::platform::webots::SensorTimeStep> sensor_time_steps;
        ::std::vector<::message::platform::webots::CameraQuality> camera_qualities;
        ::std::vector<::message::platform::webots::CameraExposure> camera_exposures;
        ::message::platform::webots::OptimisationCommand optimisation_command;
    };

}  // namespace message::platform::webots

#endif  // MESSAGE_PLATFORM_WEBOTS_MESSAGES_HPP
