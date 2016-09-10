
#include <armadillo>
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/matrix/Rotation3D.h"
#include <string>
#include <iostream>

#ifndef MOCAP_RECORDER_H
#define MOCAP_RECORDER_H

class MocapRecorder{
	std::string folder;
public:
	MocapRecorder(std::string foldername){
		folder = foldername;
	}

	bool saveFrame(const std::vector<utility::math::matrix::Transform3D>& pose);
};

#endif