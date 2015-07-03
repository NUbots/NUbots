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
#include "messages/support/nubugger/proto/DataPoint.pb.h"
#include "messages/support/nubugger/proto/DrawObjects.pb.h"
#include "messages/vision/proto/VisionObject.pb.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/geometry/RotatedRectangle.h"
#include "utility/math/geometry/Circle.h"
#include "utility/math/matrix/Transform2D.h"

namespace utility {
namespace nubugger {
    using utility::math::geometry::RotatedRectangle;
    using utility::math::geometry::Circle;
    using utility::math::matrix::Transform2D;
    using messages::support::nubugger::proto::DrawObjects;

    namespace {

        using messages::support::nubugger::proto::DataPoint;
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
    inline std::unique_ptr<messages::support::nubugger::proto::DataPoint> graph(std::string label, Values... values) {
        auto dataPoint = std::make_unique<DataPoint>();
        dataPoint->set_label(label);
        dataPoint->set_type(DataPoint::FLOAT_LIST);
        buildGraph(*dataPoint, values...);
        return dataPoint;
    }

    inline std::unique_ptr<messages::support::nubugger::proto::DataPoint> graph(std::string label, Rotation3D rotation) {
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
        object->set_shape(messages::support::nubugger::proto::DrawObject::ARROW);
        object->set_timeout(timeout);

        auto* objPosition = object->mutable_position();
        objPosition->set_x(position[0]);
        objPosition->set_y(position[1]);
        objPosition->set_z(position[2]);

        auto* objDirection = object->mutable_direction();
        objDirection->set_x(direction[0]);
        objDirection->set_y(direction[1]);
        objDirection->set_z(direction[2]);

        object->set_length(length);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawArrow(std::string name, arma::vec3 position, arma::vec3 target, float timeout = TIMEOUT) {

        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(messages::support::nubugger::proto::DrawObject::ARROW);
        object->set_timeout(timeout);

        auto* objPosition = object->mutable_position();
        objPosition->set_x(position[0]);
        objPosition->set_y(position[1]);
        objPosition->set_z(position[2]);

        auto* objTarget = object->mutable_target();
        objTarget->set_x(target[0]);
        objTarget->set_y(target[1]);
        objTarget->set_z(target[2]);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawBox(std::string name, arma::vec3 position, float width, float height, float depth, float timeout = TIMEOUT) {

        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(messages::support::nubugger::proto::DrawObject::BOX);
        object->set_timeout(timeout);

        auto* objPosition = object->mutable_position();
        objPosition->set_x(position[0]);
        objPosition->set_y(position[1]);
        objPosition->set_z(position[2]);

        object->set_width(width);
        object->set_height(height);
        object->set_depth(depth);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawCircle(std::string name, arma::vec3 position, arma::vec3 rotation, float width, float height, float timeout = TIMEOUT) {

        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(messages::support::nubugger::proto::DrawObject::CIRCLE);
        object->set_timeout(timeout);

        auto* objPosition = object->mutable_position();
        objPosition->set_x(position[0]);
        objPosition->set_y(position[1]);
        objPosition->set_z(position[2]);

        auto* objRotation = object->mutable_rotation();
        objRotation->set_x(rotation[0]);
        objRotation->set_y(rotation[1]);
        objRotation->set_z(rotation[2]);

        object->set_width(width);
        object->set_height(height);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawCircle(std::string name, Circle circle, float z = 0, arma::vec3 color = {1,1,0}, float timeout = TIMEOUT) {
        auto drawObjects = drawCircle(
            name,
            {circle.centre(0), circle.centre(1), z},
            {0,0,0},
            circle.radius * 2,
            circle.radius * 2,
            timeout);
        auto* object = drawObjects->mutable_objects(0);
        auto* objColor = object->mutable_color();
        objColor->set_x(color[0]);
        objColor->set_y(color[1]);
        objColor->set_z(color[2]);
        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawCylinder(std::string name, arma::vec3 position, arma::vec3 rotation, float topRadius, float bottomRadius, float height, float timeout = TIMEOUT) {

        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(messages::support::nubugger::proto::DrawObject::CYLINDER);
        object->set_timeout(timeout);

        auto* objPosition = object->mutable_position();
        objPosition->set_x(position[0]);
        objPosition->set_y(position[1]);
        objPosition->set_z(position[2]);

        auto* objRotation = object->mutable_rotation();
        objRotation->set_x(rotation[0]);
        objRotation->set_y(rotation[1]);
        objRotation->set_z(rotation[2]);

        object->set_height(height);
        object->set_top_radius(topRadius);
        object->set_bottom_radius(bottomRadius);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawPyramid(std::string name, arma::vec3 position, arma::vec3 rotation, float height, float faces, float timeout = TIMEOUT) {

        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(messages::support::nubugger::proto::DrawObject::PYRAMID);
        object->set_timeout(timeout);

        auto* objPosition = object->mutable_position();
        objPosition->set_x(position[0]);
        objPosition->set_y(position[1]);
        objPosition->set_z(position[2]);

        auto* objRotation = object->mutable_rotation();
        objRotation->set_x(rotation[0]);
        objRotation->set_y(rotation[1]);
        objRotation->set_z(rotation[2]);

        object->set_height(height);
        object->set_faces(faces);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawRectangle(std::string name, arma::vec3 position, float height, float length, float timeout = TIMEOUT) {

        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(messages::support::nubugger::proto::DrawObject::RECTANGLE);
        object->set_timeout(timeout);

        auto* objPosition = object->mutable_position();
        objPosition->set_x(position[0]);
        objPosition->set_y(position[1]);
        objPosition->set_z(position[2]);

        object->set_height(height);
        object->set_length(length);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawRectangle(std::string name, RotatedRectangle rect, float z = 0.08, float timeout = TIMEOUT) {

        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(messages::support::nubugger::proto::DrawObject::RECTANGLE);
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

    inline std::unique_ptr<DrawObjects> drawRectangle(std::string name, RotatedRectangle rect, arma::vec3 color, float timeout = TIMEOUT) {
        auto drawObjects = drawRectangle(name, rect, timeout);
        auto* object = drawObjects->mutable_objects(0);
        auto* objColor = object->mutable_color();
        objColor->set_x(color[0]);
        objColor->set_y(color[1]);
        objColor->set_z(color[2]);
        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawSphere(std::string name, arma::vec3 position, float radius, float timeout = TIMEOUT) {

        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(messages::support::nubugger::proto::DrawObject::SPHERE);
        object->set_timeout(timeout);

        auto* objPosition = object->mutable_position();
        objPosition->set_x(position[0]);
        objPosition->set_y(position[1]);
        objPosition->set_z(position[2]);

        object->set_radius(radius);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawSphere(std::string name, arma::vec3 position, float radius, arma::vec3 color, float timeout = TIMEOUT) {
        auto drawObjects = drawSphere(name, position, radius, timeout);
        auto* object = drawObjects->mutable_objects(0);
        auto* objColor = object->mutable_color();
        objColor->set_x(color[0]);
        objColor->set_y(color[1]);
        objColor->set_z(color[2]);
        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawTree(std::string name, std::vector<arma::vec> positions, std::vector<uint> parentIndices, float line_width, arma::vec3 color, float timeout = TIMEOUT) {

        auto drawObjects = std::make_unique<DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(messages::support::nubugger::proto::DrawObject::POLYLINE);
        object->set_timeout(timeout);
        object->set_width(line_width);
        auto* objColor = object->mutable_color();
        objColor->set_x(color[0]);
        objColor->set_y(color[1]);
        objColor->set_z(color[2]);

        for (uint i = 0; i < positions.size(); i++) {
            auto* objNode = object->add_path();

            auto* nodeVertex = objNode->mutable_position();
            nodeVertex->set_x(positions[i](0));
            nodeVertex->set_y(positions[i](1));

            objNode->set_parent_index(parentIndices[i]);
        }

        return std::move(drawObjects);
    }

    inline std::unique_ptr<DrawObjects> drawPolyline(std::string name, std::vector<arma::vec> positions, float line_width, arma::vec3 color, float timeout = TIMEOUT) {

        std::vector<uint> parentIndices;
        parentIndices.reserve(positions.size());
        for (uint i = 0; i < positions.size(); i++) {
            parentIndices.push_back(std::max(0, int(i) - 1));
        }
        return drawTree(name, positions, parentIndices, line_width, color, timeout);

    }

    inline std::unique_ptr<DrawObjects> drawPath(std::string name, std::vector<Transform2D> states, float line_width, arma::vec3 color, float timeout = TIMEOUT) {
        std::vector<arma::vec> positions;

        for (auto state : states) {
            positions.push_back(state.xy());
        }

        return utility::nubugger::drawPolyline(name, positions, line_width, color, timeout);
    }

    inline std::unique_ptr<messages::vision::proto::VisionObject> drawVisionLines(std::vector<std::tuple<arma::ivec2, arma::ivec2, arma::vec4>> lines) {

        auto visionObject = std::make_unique<messages::vision::proto::VisionObject>();

        visionObject->set_type(messages::vision::proto::VisionObject::LINE);
        visionObject->set_camera_id(0); // TODO

        for (const auto& line : lines) {
            auto* objLine = visionObject->add_line();

            auto* start = objLine->mutable_start();
            start->set_x(std::get<0>(line)[0]);
            start->set_y(std::get<0>(line)[1]);

            auto* end = objLine->mutable_end();
            end->set_x(std::get<1>(line)[0]);
            end->set_y(std::get<1>(line)[1]);
        }

        return std::move(visionObject);

    }

    inline std::unique_ptr<messages::vision::proto::VisionObject> drawVisionLine(arma::ivec2 start, arma::ivec2 end, arma::vec4 color = arma::vec4({1, 0, 0, 1})) {
        return drawVisionLines({std::make_tuple(start, end, color)});
    }

}
}

#endif
