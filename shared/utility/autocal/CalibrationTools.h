//////////////////////////////////////////////////////////////////////////////
//	  Author: Jake Fountain 2015
//    
//
//////////////////////////////////////////////////////////////////////////////

#ifndef AUTOCAL_CALIBRATION_TOOLS_H
#define AUTOCAL_CALIBRATION_TOOLS_H

#include <armadillo>
#include <math.h>
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/geometry/UnitQuaternion.h"

namespace autocal{

	class CalibrationTools{
	public:
		static int kroneckerDelta(int i, int j);

		static arma::mat33 crossMatrix(const arma::vec3& v);

		static bool solveWithSVD(const arma::mat& A, const arma::vec& b, arma::mat& x);

		static arma::mat unvectorize(arma::mat v, int n_rows);

		static std::pair<arma::vec3,arma::vec3> getTranslationComponent(const std::vector<utility::math::matrix::Transform3D>& samplesA, const std::vector<utility::math::matrix::Transform3D>& samplesB, const utility::math::matrix::Rotation3D& Ry, bool& success);

		static std::pair<utility::math::matrix::Transform3D, utility::math::matrix::Transform3D> solveZhuang1994(const std::vector<utility::math::matrix::Transform3D>& samplesA,const std::vector<utility::math::matrix::Transform3D>& samplesB, bool& success);
		static std::pair<utility::math::matrix::Transform3D, utility::math::matrix::Transform3D> solveKronecker_Shah2013(const std::vector<utility::math::matrix::Transform3D>& samplesA,const std::vector<utility::math::matrix::Transform3D>& samplesB, bool& success);
		static std::pair<utility::math::matrix::Transform3D, utility::math::matrix::Transform3D> solveClosedForm_Dornaika1998(const std::vector<utility::math::matrix::Transform3D>& samplesA,const std::vector<utility::math::matrix::Transform3D>& samplesB, bool& success);

	};
}
#endif
