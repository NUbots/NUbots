#ifndef MESSAGE_CONVERSION_PROTO_TRANSFORM_H
#define MESSAGE_CONVERSION_PROTO_TRANSFORM_H

#include "utility/conversion/proto_matrix.h"

/**
 * @brief Functions to convert the transform classes
 */
protobuf::message::Transform2D& operator<<(protobuf::message::Transform2D& proto,
                                           const utility::math::matrix::Transform2D& transform) {
    *proto.mutable_transform() << static_cast<::message::math::vec3>(transform);
    return proto;
}

utility::math::matrix::Transform2D& operator<<(utility::math::matrix::Transform2D& transform,
                                               const protobuf::message::Transform2D& proto) {
    arma::vec3 t;
    t << proto.transform();
    transform = t;
    return transform;
}

protobuf::message::Transform3D& operator<<(protobuf::message::Transform3D& proto,
                                           const utility::math::matrix::Transform3D& transform) {
    *proto.mutable_transform() << static_cast<arma::mat44>(transform);
    return proto;
}

utility::math::matrix::Transform3D& operator<<(utility::math::matrix::Transform3D& transform,
                                               const protobuf::message::Transform3D& proto) {
    arma::mat44 t;
    t << proto.transform();
    transform = t;
    return transform;
}

/**
 * @brief Functions to convert the rotation classes
 */
protobuf::message::Rotation2D& operator<<(protobuf::message::Rotation2D& proto,
                                          const utility::math::matrix::Rotation2D& transform) {
    *proto.mutable_rotation() << static_cast<arma::mat22>(transform);
    return proto;
}

utility::math::matrix::Rotation2D& operator<<(utility::math::matrix::Rotation2D& transform,
                                              const protobuf::message::Rotation2D& proto) {
    arma::mat22 t;
    t << proto.rotation();
    transform = t;
    return transform;
}

protobuf::message::Rotation3D& operator<<(protobuf::message::Rotation3D& proto,
                                          const utility::math::matrix::Rotation3D& transform) {
    *proto.mutable_rotation() << static_cast<arma::mat33>(transform);
    return proto;
}

utility::math::matrix::Rotation3D& operator<<(utility::math::matrix::Rotation3D& transform,
                                              const protobuf::message::Rotation3D& proto) {
    arma::mat33 t;
    t << proto.rotation();
    transform = t;
    return transform;
}

#endif  // MESSAGE_CONVERSION_PROTO_TRANSFORM_H
