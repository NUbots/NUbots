#ifndef MESSAGE_CONVERSION_PROTO_TIME_H
#define MESSAGE_CONVERSION_PROTO_TIME_H

#include <chrono>
#include <nuclear_bits/clock.hpp>

#include <google/protobuf/timestamp.pb.h>
#include <google/protobuf/duration.pb.h>

template <typename Clock>
google::protobuf::Timestamp& operator<< (google::protobuf::Timestamp& proto, const std::chrono::time_point<Clock>& t) {

    // Get the epoch timestamp
    auto d = t.time_since_epoch();

    // Get our seconds and the remainder nanoseconds
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(d);
    auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(d - seconds);

    // Set our seconds and nanoseconds
    proto.set_seconds(seconds.count());
    proto.set_nanos(nanos.count());
    return proto;
}

template <typename Clock>
std::chrono::time_point<Clock>& operator<< (std::chrono::time_point<Clock>& t, const google::protobuf::Timestamp& proto) {
    // Get our seconds and nanos in c++ land
    auto seconds = std::chrono::seconds(proto.seconds());
    auto nanos = std::chrono::nanoseconds(proto.nanos());

    // Make a timestamp out of the summation of them
    t = std::chrono::time_point<Clock>(seconds + nanos);
    return t;
}

template <typename Rep, typename Period>
google::protobuf::Duration& operator<< (google::protobuf::Duration& proto, const std::chrono::duration<Rep, Period>& d) {

    // Get our seconds and the remainder nanoseconds
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(d);
    auto nanos = std::chrono::duration_cast<std::chrono::seconds>(d - seconds);

    // Set our seconds and nanoseconds
    proto.set_seconds(seconds.count());
    proto.set_nanos(nanos.count());

    return proto;
}

template <typename Rep, typename Period>
std::chrono::duration<Rep, Period>& operator<< (std::chrono::duration<Rep, Period>& d, const google::protobuf::Duration& proto) {

    // Get our seconds and nanos in c++ land
    auto seconds = std::chrono::seconds(proto.seconds());
    auto nanos = std::chrono::nanoseconds(proto.nanos());

    // Make a duration out of the summation of them
    d = std::chrono::duration_cast<std::chrono::duration<Rep, Period>>(seconds + nanos);
    return d;
}

#endif  // MESSAGE_CONVERSION_PROTO_TIME_H
