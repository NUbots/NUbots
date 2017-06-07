#ifndef MESSAGE_CONVERSION_PROTO_TRANSFORM_H
#define MESSAGE_CONVERSION_PROTO_TRANSFORM_H

#include "utility/conversion/proto_matrix.h"

/**
 * @brief Functions to convert the transform classes
 */
protobuf::message::Transform2D& operator<< (protobuf::message::Transform2D& proto, const utility::math::matrix::Transform2D& transform) {
    *proto.mutable_transform() << static_cast<::message::math::vec3>(transform);
    return proto;
}

utility::math::matrix::Transform2D& operator<< (utility::math::matrix::Transform2D& transform, const protobuf::message::Transform2D& proto) {
    Eigen::Vector3d t;
    t << proto.transform();
    transform = t;
    return transform;
}

protobuf::message::Transform3D& operator<< (protobuf::message::Transform3D& proto, const utility::math::matrix::Transform3D& transform) {
    *proto.mutable_transform() << static_cast<Eigen::Matrix4d>(transform);
    return proto;
}

utility::math::matrix::Transform3D& operator<< (utility::math::matrix::Transform3D& transform, const protobuf::message::Transform3D& proto) {
    Eigen::Matrix4d t;
    t << proto.transform();
    transform = t;
    return transform;
}

/**
 * @brief Functions to convert the rotation classes
 */
protobuf::message::Rotation2D& operator<< (protobuf::message::Rotation2D& proto, const utility::math::matrix::Rotation2D& transform) {
    *proto.mutable_rotation() << static_cast<Eigen::Matrix2d>(transform);
    return proto;
}

utility::math::matrix::Rotation2D& operator<< (utility::math::matrix::Rotation2D& transform, const protobuf::message::Rotation2D& proto) {
    Eigen::Matrix2d t;
    t << proto.rotation();
    transform = t;
    return transform;
}

protobuf::message::Rotation3D& operator<< (protobuf::message::Rotation3D& proto, const utility::math::matrix::Rotation3D& transform) {
    *proto.mutable_rotation() << static_cast<Eigen::Matrix3d>(transform);
    return proto;
}

utility::math::matrix::Rotation3D& operator<< (utility::math::matrix::Rotation3D& transform, const protobuf::message::Rotation3D& proto) {
    Eigen::Matrix3d t;
    t << proto.rotation();
    transform = t;
    return transform;
}

#endif  // MESSAGE_CONVERSION_PROTO_TRANSFORM_H
