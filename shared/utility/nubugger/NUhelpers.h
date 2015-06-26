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
#include "utility/math/matrix/Rotation3D.h"

namespace utility {
namespace nubugger {
    namespace {

        using messages::support::nubugger::proto::DataPoint;
        using utility::math::matrix::Rotation3D;

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

    inline std::unique_ptr<messages::support::nubugger::proto::DrawObjects> drawArrow(std::string name, arma::vec position, arma::vec direction, float length) {

        auto drawObjects = std::make_unique<messages::support::nubugger::proto::DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(messages::support::nubugger::proto::DrawObject::ARROW);

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

    inline std::unique_ptr<messages::support::nubugger::proto::DrawObjects> drawArrow(std::string name, arma::vec position, arma::vec target) {

        auto drawObjects = std::make_unique<messages::support::nubugger::proto::DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(messages::support::nubugger::proto::DrawObject::ARROW);

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

    inline std::unique_ptr<messages::support::nubugger::proto::DrawObjects> drawBox(std::string name, arma::vec position, float width, float height, float depth) {

        auto drawObjects = std::make_unique<messages::support::nubugger::proto::DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(messages::support::nubugger::proto::DrawObject::BOX);

        auto* objPosition = object->mutable_position();
        objPosition->set_x(position[0]);
        objPosition->set_y(position[1]);
        objPosition->set_z(position[2]);

        object->set_width(width);
        object->set_height(height);
        object->set_depth(depth);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<messages::support::nubugger::proto::DrawObjects> drawCircle(std::string name, arma::vec position, arma::vec rotation, float width, float height) {

        auto drawObjects = std::make_unique<messages::support::nubugger::proto::DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(messages::support::nubugger::proto::DrawObject::CIRCLE);

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

    inline std::unique_ptr<messages::support::nubugger::proto::DrawObjects> drawCylinder(std::string name, arma::vec position, arma::vec rotation, float topRadius, float bottomRadius, float height) {

        auto drawObjects = std::make_unique<messages::support::nubugger::proto::DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(messages::support::nubugger::proto::DrawObject::CYLINDER);

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

    inline std::unique_ptr<messages::support::nubugger::proto::DrawObjects> drawPyramid(std::string name, arma::vec position, arma::vec rotation, float height, float faces) {

        auto drawObjects = std::make_unique<messages::support::nubugger::proto::DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(messages::support::nubugger::proto::DrawObject::PYRAMID);

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

    inline std::unique_ptr<messages::support::nubugger::proto::DrawObjects> drawRectangle(std::string name, arma::vec position, float height, float length) {

        auto drawObjects = std::make_unique<messages::support::nubugger::proto::DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(messages::support::nubugger::proto::DrawObject::RECTANGLE);

        auto* objPosition = object->mutable_position();
        objPosition->set_x(position[0]);
        objPosition->set_y(position[1]);
        objPosition->set_z(position[2]);

        object->set_height(height);
        object->set_length(length);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<messages::support::nubugger::proto::DrawObjects> drawSphere(std::string name, arma::vec position, float radius) {

        auto drawObjects = std::make_unique<messages::support::nubugger::proto::DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(messages::support::nubugger::proto::DrawObject::SPHERE);

        auto* objPosition = object->mutable_position();
        objPosition->set_x(position[0]);
        objPosition->set_y(position[1]);
        objPosition->set_z(position[2]);

        object->set_radius(radius);

        return std::move(drawObjects);
    }

    inline std::unique_ptr<messages::support::nubugger::proto::DrawObjects> drawTree(std::string name, std::vector<arma::vec> positions, std::vector<int> parentIndices) {

        auto drawObjects = std::make_unique<messages::support::nubugger::proto::DrawObjects>();
        auto* object = drawObjects->add_objects();
        object->set_name(name);
        object->set_shape(messages::support::nubugger::proto::DrawObject::POLYLINE);

        
        for (unsigned int i = 0; i < positions.size(); i++) {
            auto* objNode = object->add_path();

            auto* nodeVertex = objNode->mutable_position();
            nodeVertex->set_x(positions[i](0));
            nodeVertex->set_y(positions[i](1));

            objNode->set_parentindex(parentIndices[i]);
        }

        return std::move(drawObjects);
    }

}
}

#endif
