/*
 * This file is part of the NUbots Codebase.
 * @author Jake Fountain
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
 #include <armadillo>


namespace modules {
namespace support {

	class VirtualVisionObject{
	public:
		VirtualVisionObject(arma::vec2 position_, float height_){
			position = position_;
			height = height_;
		}
		~VirtualVisionObject(){}

		arma::vec2 position;
		float height;

		VisionObject detectBasicObject(cam_params, robot_pose){
			vClass obj;
			return obj;
		}

		virtual void std::unique_ptr<VisionObject> detect(cam_params, robot_pose) = 0;
	};

	class VirtualGoal : VirtualVisionObject{

	};

}
}