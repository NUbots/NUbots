/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef NUHELPERS_H
#define NUHELPERS_H

#include <nuclear>
#include <armadillo>
#include "message/support/nubugger/proto/DataPoint.pb.h"
#include "message/support/nubugger/proto/DrawObjects.pb.h"
#include "message/vision/proto/VisionObjects.pb.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/geometry/RotatedRectangle.h"
#include "utility/math/geometry/Circle.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/support/proto_armadillo.h"

namespace utility {
namespace nubugger {
    using utility::math::geometry::RotatedRectangle;
    
    using utility::math::geometry::Circle;
    using utility::math::matrix::Transform2D;
    using message::support::nubugger::proto::DrawObjects;

    namespace {

        using message::support::nubugger::proto::DataPoint;
        using utility::math::matrix::Rotation3D;

        constexpr float TIMEOUT = 2.5;

        template<typename T>
        struct is_iterable {
            private:
                typedef std::true_type yes;
                typedef std::false_type no;

                template<typename U> static auto test_begin(int) -> decltype(std::declval<U>().begin(), yes());
                template<typename> static no test_begin(...);

                template<typename U> static auto test_end(int) -> decltype(std::declval<U>().end(), yes());
                template<typename> static no test_end(...);
            public:
                static constexpr bool value = std::is_same<decltype(test_begin<T>(0)), yes>::value
                                              && std::is_same<decltype(test_end<T>(0)), yes>::value;
        };

        inline void buildGraph(DataPoint&) {
        }

        template<typename First, typename... Remainder>
        typename std::enable_if<!is_iterable<First>::value>::type buildGraph(DataPoint& dataPoint, First first, Remainder... remainder) {
            dataPoint.add_value(first);
            buildGraph(dataPoint, remainder...);
        }

        template<typename First, typename... Remainder>
        typename std::enable_if<is_iterable<First>::value>::type buildGraph(DataPoint& dataPoint, First first, Remainder... remainder) {
            for (const auto& value : first) {
                dataPoint.add_value(value);
            }
            buildGraph(dataPoint, remainder...);
        }
    }

    template<typename... Values>
    inline std::unique_ptr<message::support::nubugger::proto::DataPoint> graph(std::string label, Values... values) {
        auto dataPoint = std::make_unique<DataPoint>();
        dataPoint->set_label(label);
        dataPoint->set_type(DataPoint::FLOAT_LIST);
        buildGraph(*dataPoint, values...);
        return dataPoint;
    }

    inline std::unique_ptr<message::support::nubugger::proto::DataPoint> graph(std::string label, Rotation3D rotation) {
        auto dataPoint = std::make_unique<DataPoint>();
        dataPoint->set_label(label);
        dataPoint->set_type(DataPoint::ROTATION_3D);
        for (const auto& value : rotation) {
            dataPoint->add_value(value);
        }
        return dataPoint;
    }

    inline std::unique_ptr<DrawObjects> drawArrow(std::string name, arma::vec3 position, float length, arma::vec3 direction, float timeout = TIMEOUT) {

        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(message::support::nubugger::proto::DrawObject::ARROW);
        object->set_timeout(timeout);

        *object->mutable_position() << position;
        *object->mutable_direction() << direction;
        object->set_length(length);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawArrow(std::string name, arma::vec3 position, arma::vec3 target, float timeout = TIMEOUT) {
        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(message::support::nubugger::proto::DrawObject::ARROW);
        object->set_timeout(timeout);
        *object->mutable_position() << position;
        *object->mutable_target() << target;
        return std::move(drawObjects);
    }


    inline std::unique_ptr<DrawObjects> drawArrow(std::string name, arma::vec2 position, arma::vec2 target, float timeout = TIMEOUT) {
        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(message::support::nubugger::proto::DrawObject::ARROW);
        object->set_timeout(timeout);
        *object->mutable_position() << arma::vec3({position(0), position(1), 0});
        *object->mutable_target() <<  arma::vec3({target(0), target(1), 0});
        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawArrow(std::string name, Transform2D position, arma::vec3 colour, float length, float timeout = TIMEOUT) {

        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(message::support::nubugger::proto::DrawObject::ARROW);
        object->set_timeout(timeout);

        *object->mutable_position() << arma::vec3({position.x(), position.y(), 0});;
        *object->mutable_direction() << arma::vec3({std::cos(position.angle()), std::sin(position.angle()), 0});
        *object->mutable_colour() << colour;

        object->set_length(length);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawBox(std::string name, arma::vec3 position, float width, float height, float depth, float timeout = TIMEOUT) {

        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(message::support::nubugger::proto::DrawObject::BOX);
        object->set_timeout(timeout);

        *object->mutable_position() << position;

        object->set_width(width);
        object->set_height(height);
        object->set_depth(depth);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawCircle(std::string name, arma::vec3 position, arma::vec3 rotation, float width, float height, float timeout = TIMEOUT) {

        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(message::support::nubugger::proto::DrawObject::CIRCLE);
        object->set_timeout(timeout);

        *object->mutable_position() << position;
        *object->mutable_rotation() << rotation;

        object->set_width(width);
        object->set_height(height);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawCircle(std::string name, Circle circle, float z = 0, arma::vec3 colour = {1,1,0}, float timeout = TIMEOUT) {
        auto drawObjects = drawCircle(
            name,
            {circle.centre(0), circle.centre(1), z},
            {0,0,0},
            circle.radius * 2,
            circle.radius * 2,
            timeout);

        auto* object = drawObjects->mutable_objects(0);
        *object->mutable_colour() << colour;

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawCylinder(std::string name, arma::vec3 position, arma::vec3 rotation, float topRadius, float bottomRadius, float height, float timeout = TIMEOUT) {

        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(message::support::nubugger::proto::DrawObject::CYLINDER);
        object->set_timeout(timeout);

        *object->mutable_position() << position;
        *object->mutable_rotation() << rotation;

        object->set_height(height);
        object->set_top_radius(topRadius);
        object->set_bottom_radius(bottomRadius);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawPyramid(std::string name, arma::vec3 position, arma::vec3 rotation, float height, float faces, float timeout = TIMEOUT) {

        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(message::support::nubugger::proto::DrawObject::PYRAMID);
        object->set_timeout(timeout);

        *object->mutable_position() << position;
        *object->mutable_rotation() << rotation;

        object->set_height(height);
        object->set_faces(faces);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawRectangle(std::string name, arma::vec3 position, float height, float length, float timeout = TIMEOUT) {

        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(message::support::nubugger::proto::DrawObject::RECTANGLE);
        object->set_timeout(timeout);

        *object->mutable_position() << position;

        object->set_height(height);
        object->set_length(length);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawRectangle(std::string name, RotatedRectangle rect, float z = 0.08, float timeout = TIMEOUT) {

        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(message::support::nubugger::proto::DrawObject::RECTANGLE);
        object->set_timeout(timeout);

        auto* objPosition = object->mutable_position();
        objPosition->set_x(rect.getPosition()(0));
        objPosition->set_y(rect.getPosition()(1));
        objPosition->set_z(z);

        auto* objRotation = object->mutable_rotation();
        // objRotation->set_x(std::cos(rect.getRotation()));
        // objRotation->set_y(std::sin(rect.getRotation()));
        objRotation->set_x(0);
        objRotation->set_y(0);
        objRotation->set_z(rect.getRotation());

        object->set_length(rect.getSize()(0));
        object->set_width(rect.getSize()(1));

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawRectangle(std::string name, RotatedRectangle rect, arma::vec3 colour, float z = 0.08, float timeout = TIMEOUT) {
        auto drawObjects = drawRectangle(name, rect, z, timeout);
        auto* object = drawObjects->mutable_objects(0);
        *object->mutable_colour() << colour;
        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawSphere(std::string name, arma::vec3 position, float radius, float timeout = TIMEOUT) {

        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(message::support::nubugger::proto::DrawObject::SPHERE);
        object->set_timeout(timeout);

        *object->mutable_position() << position;

        object->set_radius(radius);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawSphere(std::string name, arma::vec3 position, float radius, arma::vec3 colour, float timeout = TIMEOUT) {
        auto drawObjects = drawSphere(name, position, radius, timeout);
        auto* object = drawObjects->mutable_objects(0);
        *object->mutable_colour() << colour;
        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawTree(std::string name, std::vector<arma::vec> positions, std::vector<uint> parentIndices, float line_width, arma::vec3 colour, float timeout = TIMEOUT) {

        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(message::support::nubugger::proto::DrawObject::POLYLINE);
        object->set_timeout(timeout);
        object->set_width(line_width);

        *object->mutable_colour() << colour;

        for (uint i = 0; i < positions.size(); i++) {
            auto* objNode = object->add_path();

            auto* nodeVertex = objNode->mutable_position();
            nodeVertex->set_x(positions[i](0));
            nodeVertex->set_y(positions[i](1));

            objNode->set_parent_index(parentIndices[i]);
        }

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawPolyline(std::string name, std::vector<arma::vec> positions, float line_width, arma::vec3 colour, float timeout = TIMEOUT) {

        std::vector<uint> parentIndices;
        parentIndices.reserve(positions.size());
        for (uint i = 0; i < positions.size(); i++) {
            parentIndices.push_back(std::max(0, int(i) - 1));
        }
        return drawTree(name, positions, parentIndices, line_width, colour, timeout);

    }

    inline std::unique_ptr<DrawObjects> drawPath(std::string name, std::vector<Transform2D> states, float line_width, arma::vec3 colour, float timeout = TIMEOUT) {
        std::vector<arma::vec> positions;

        for (auto state : states) {
            positions.push_back(state.xy());
        }

        return utility::nubugger::drawPolyline(name, positions, line_width, colour, timeout);
    }

    inline std::unique_ptr<message::vision::proto::VisionObject> drawVisionLines(std::vector<std::tuple<arma::ivec2, arma::ivec2, arma::vec4>> lines) {

        auto visionObject = std::make_unique<message::vision::proto::VisionObject>();

        visionObject->set_type(message::vision::proto::VisionObject::LINE);
        visionObject->set_camera_id(0); // TODO

        for (const auto& line : lines) {
            auto* objLine = visionObject->add_line();

            *objLine->mutable_start() << std::get<0>(line);
            *objLine->mutable_end() << std::get<1>(line);
            *objLine->mutable_colour() << std::get<2>(line);
        }

        return std::move(visionObject);

    }

    inline std::unique_ptr<message::vision::proto::VisionObject> drawVisionLines(std::vector<std::pair<arma::ivec2, arma::ivec2>> lines, arma::vec4 colour = arma::vec4({1, 1, 1, 1})) {
        std::vector<std::tuple<arma::ivec2, arma::ivec2, arma::vec4>> colouredLines;
        colouredLines.reserve(lines.size());
        for (auto const line : lines) {
            colouredLines.push_back(std::make_tuple(line.first, line.second, colour));
        }
        return drawVisionLines(colouredLines);
    }

    inline std::unique_ptr<message::vision::proto::VisionObject> drawVisionLine(arma::ivec2 start, arma::ivec2 end, arma::vec4 colour = arma::vec4({1, 1, 1, 1})) {
        return drawVisionLines({std::make_tuple(start, end, colour)});
    }

}
}

#endif
