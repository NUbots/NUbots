//////////////////////////////////////////////////////////////////////////////
//	  Author: Jake Fountain 2015
//
/// \file CalibrationTools.cpp
/// \brief CPP file for CalibrationTools.h
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <math.h>
#include <limits>
#include "CalibrationTools.h"

namespace autocal{

	using utility::math::geometry::UnitQuaternion;
	using utility::math::matrix::Rotation3D;
	using utility::math::matrix::Transform3D;

	int CalibrationTools::kroneckerDelta(int i, int j){
		return (i == j);
	}

	template <typename T> int sgn(T val) {
	    return (T(0) < val) - (val < T(0));
	}

	/*
	Returns matrix M(v) such that for any vector x, cross(v,x) = dot(M(v),x)
	*/
	Eigen::Matrix3d CalibrationTools::crossMatrix(const Eigen::Vector3d& v){
		Eigen::Matrix3d omega;
		omega <<  0 <<    -v[2] <<  v[1] << arma::endr
			  << v[2] <<     0  << -v[0] << arma::endr
			  << -v[1]<<   v[0] <<    0  << arma::endr;
		return omega;
	}

	/*
	 * Returns matrix of vectorised matrix v
	*/
	arma::mat CalibrationTools::unvectorize(arma::mat v, int n_rows){
		int n_cols = 0;
		if(v.size() % n_rows != 0){
			std::cout << "CalibrationTools::unvectorize - vector size is not divisible by number of rows: v.size() = " <<  v.size() << ", n_rows = " << n_rows << std::endl;
		} else {
			n_cols = v.size() / n_rows;
		}
		arma::mat M = v;
		M.reshape(n_rows,n_cols);
		return M;
	}

	//
	// void CalibrationTools::checkOrthonormal(M){

	// 	if M.shape[0] == 4:
	// 		if numpy.linalg.norm(M[3,:3]) != 0:
	// 			print "\n\n\n\n\n\n\nBottom row of matrix non-zero ", numpy.linalg.norm(M[3,:3]), "\n\n\n\n\n\n\n"
	// 			return False
	// 	for i in range(3):
	// 		for j in range(3):
	// 			if not (numpy.allclose(dot(M[:3,i],M[:3,j]), kroneckerDelta(i,j))):
	// 				print "\n\n\n\n\n\n\nColumn ", i, " and Column ", j, " are not orthonormal: dotprod = ", dot(M[:3,i],M[:3,j]), "\n\n\n\n\n\n\n"
	// 				return False
	// 	return True
	// }

	/*
		Computes least squares solution x to Ax = b using the psuedoinverse
	*/
	bool CalibrationTools::solveWithSVD(const arma::mat& A, const arma::vec& b, arma::mat& x){
		if (A.n_rows != b.n_rows){
			throw("Problem badly formulated!");
		}
		//Compute pseudo inverse
		arma::mat pinvA;
		bool success = arma::pinv(pinvA,A);
		//Compute x in Ax=b
		if(success) x = pinvA * b;

		auto error = A*x-b;
		// std::cout << "SVD error: A*x - b = \n" << error.t() << " size = " << error.norm() << std::endl;

		//Return whether or not the SVD was performed correctly
		return success;
	}

	std::pair<Eigen::Vector3d,Eigen::Vector3d> CalibrationTools::getTranslationComponent(const std::vector<Transform3D>& samplesA, const std::vector<Transform3D>& samplesB,const Rotation3D& Ry, bool& success){
		arma::mat combinedF;
		Eigen::VectorXd combinedD;

		for (int i = 0; i < samplesA.size(); i++){
			Rotation3D RA = samplesA[i].rotation();
			Eigen::Vector3d pA = samplesA[i].translation();
			Eigen::Vector3d pB = samplesB[i].translation();

			arma::mat F = arma::join_rows(RA,-Eigen::Matrix3d::Identity());

			Eigen::VectorXd D = Ry * pB - pA;

			if (i == 0){
				combinedF = F;
				combinedD = D;
			}else{
				combinedF = arma::join_cols(combinedF,F);
				combinedD = arma::join_cols(combinedD,D);
			}
		}
		Eigen::VectorXd pxpy;
		bool pxpySuccess = solveWithSVD(combinedF,combinedD,pxpy);

		if(!pxpySuccess){
			//If SVD fails, return identity
			std::cout << __FILE__ << " : " << __LINE__ << " - WARNING: SVD FAILED" << std::endl;
			success = false;
			return std::pair<Eigen::Vector3d, Eigen::Vector3d>();
		}

		std::pair<Eigen::Vector3d,Eigen::Vector3d> txty(pxpy.rows(0,2),pxpy.rows(3,5));
		return txty;
	}


		/*
		solves AX=YB for X,Y and A in sampleA, B in sampleB

		Source:
		@ARTICLE{Zhuang1994,
		author={Hanqi Zhuang and Roth, Zvi S. and Sudhakar, R.},
		journal={Robotics and Automation, IEEE Transactions on},
		title={Simultaneous robot/world and tool/flange calibration by solving homogeneous transformation equations of the form AX=YB},
		year={1994},
		month={Aug},
		volume={10},
		number={4},
		pages={549-554}
		}
		*/
	std::pair<Transform3D, Transform3D> CalibrationTools::solveZhuang1994(const std::vector<Transform3D>& samplesA,const std::vector<Transform3D>& samplesB, bool& success){
		if(samplesA.size() < 3 || samplesB.size() < 3){
			std::cout << "CalibrationTools::solveZhuang1994 - NEED MORE THAN 2 SAMPLES" << std::endl;
			throw std::domain_error("CalibrationTools::solveZhuang1994 - NEED MORE THAN 2 SAMPLES");
		}
		Transform3D X,Y;

		arma::mat combinedG;
		Eigen::VectorXd combinedC;

		float a0 = 0;
		float b0 = 0;

		Eigen::Vector3d a;
		Eigen::Vector3d b;

		for (int i = 0; i < samplesA.size(); i++){
			const Transform3D& A = samplesA[i];
			const Transform3D& B = samplesB[i];

			//Get Quaternions for rotations
			const UnitQuaternion quat_a(A.rotation());
			a0 = quat_a.real();
			a = quat_a.imaginary();

			const UnitQuaternion quat_b(B.rotation());
			b0 = quat_b.real();
			b = quat_b.imaginary();

			//Compute G in Gw = C
			if(std::fabs(a0) < 1e-10){
				std::cout << __FILE__ << " : " << __LINE__ << " - WARNING: BAD SAMPLED ROTATION - RETURNING IDENTITY" << std::endl;
				// std::cout << " A = \n" << A <<  std::endl;
				// std::cout << " A * A.i() = \n" << A * A.i() <<  std::endl;
				// std::cout << " quat_a = \n" << quat_a <<  std::endl;
				success = false;
				return std::pair<Transform3D, Transform3D>();
			}

			arma::mat G1 = a0 * Eigen::Matrix3d::Identity() + crossMatrix(a) + a*a.t() / a0;
			arma::mat G2 = -b0 * Eigen::Matrix3d::Identity() + crossMatrix(b) - a*b.t() / a0;

			arma::mat G = arma::join_rows(G1,G2);

			//Compute C in Gw = C
			Eigen::VectorXd C = b - (b0/a0) * a;

			if (i == 0){
				combinedG = G;
				combinedC = C;
			}else{
				combinedG = arma::join_cols(combinedG,G);
				combinedC = arma::join_cols(combinedC,C);
			}
		}

		Eigen::VectorXd w;
		bool wSuccess = solveWithSVD(combinedG,combinedC, w);
		if(!wSuccess){
			//If SVD fails, return identity
			std::cout << __FILE__ << " : " << __LINE__ << " - WARNING: SVD FAILED" << std::endl;
			std::cout << "combinedG = " << combinedG << std::endl;
			std::cout << "combinedC = " << combinedC << std::endl;
			int sampleNum = 0;
			for(auto& sample : samplesA){
				std::cout << " sampleA[" << sampleNum++ << "] =\n" << sample << std::endl;
			}
			sampleNum = 0;
			for(auto& sample : samplesB){
				std::cout << " sampleB[" << sampleNum++ << "] =\n" << sample << std::endl;
			}
			success = false;
			return std::pair<Transform3D, Transform3D>(X,Y);
		}

		//Compute x and y Quaternions
		UnitQuaternion x, y;
		y.real() = 1 / std::sqrt(1 + w[3]*w[3] + w[4]*w[4] + w[5]*w[5]);
		if (std::fabs(y.real())< 1e-3){
			std::cout << "\n\n\n\n\n\n\ny.real() == 0 so you need to rotate the ref base with respect to the base\n\n\n\n\n\n\n" << std::endl;
			success = false;
			return std::pair<Transform3D, Transform3D>();
		}
		y.imaginary() = y.real() * w.rows(3,5);
		x.imaginary() = y.real() * w.rows(0,2);

		float x0 = (a/a0).dot(x.rows(1,3)) + (b0/a0) * y.real() - (b/a0).dot(y.rows(1,3));
		int x_sign = x0 > 0 ? 1 : -1;
		//TODO: figure out how to handle when x is nan
		x.real() = x_sign * std::sqrt(1 - std::fmin(1, x[1]*x[1] + x[2]*x[2] + x[3]*x[3]) );
		// check:
		// check = tr.quaternion_multiply(tr.quaternion_inverse(tr.quaternion_multiply(quat_a,x)), tr.quaternion_multiply(y,quat_b))

		Rotation3D Rx(x);
		Rotation3D Ry(y);

		X.rotation() = Rx;
		Y.rotation() = Ry;

		auto translation = getTranslationComponent(samplesA, samplesB, Ry, success);

		X.translation() = translation.first;
		Y.translation() = translation.second;

		return std::pair<Transform3D, Transform3D>(X,Y);
	}


	// @article{shah_solving_2013,
	// 	title = {Solving the {Robot}-{World}/{Hand}-{Eye} {Calibration} {Problem} {Using} the {Kronecker} {Product}},
	// 	volume = {5},
	// 	issn = {1942-4302},
	// 	url = {http://dx.doi.org/10.1115/1.4024473},
	// 	doi = {10.1115/1.4024473},
	// 	abstract = {This paper constructs a separable closed-form solution to the robot-world/hand-eye calibration problem AX = YB. Qualifications and properties that determine the uniqueness of X and Y as well as error metrics that measure the accuracy of a given X and Y are given. The formulation of the solution involves the Kronecker product and the singular value decomposition. The method is compared with existing solutions on simulated data and real data. It is shown that the Kronecker method that is presented in this paper is a reliable and accurate method for solving the robot-world/hand-eye calibration problem.},
	// 	number = {3},
	// 	urldate = {2015-10-12},
	// 	journal = {Journal of Mechanisms and Robotics},
	// 	author = {Shah, Mili},
	// 	month = jun,
	// 	year = {2013},
	// 	pages = {031007--031007},
	// 	file = {Full Text PDF:/Users/jake/Library/Application Support/Zotero/Profiles/3jsx8rgb.default/zotero/storage/ZBI8MGZA/Shah - 2013 - Solving the Robot-WorldHand-Eye Calibration Probl.pdf:application/pdf}
	// }

	// solves AX=YB for X,Y and A in sampleA, B in sampleB
	std::pair<utility::math::matrix::Transform3D, utility::math::matrix::Transform3D> CalibrationTools::solveKronecker_Shah2013(const std::vector<utility::math::matrix::Transform3D>& samplesA,const std::vector<utility::math::matrix::Transform3D>& samplesB, bool& success){
		if(samplesA.size() < 3 || samplesB.size() < 3){
			std::cout << "CalibrationTools - NEED MORE THAN 2 SAMPLES" << std::endl;
			throw std::domain_error("CalibrationTools - NEED MORE THAN 2 SAMPLES");
		}
		std::pair<utility::math::matrix::Transform3D, utility::math::matrix::Transform3D> result;

		//Rotation part

		//Create kronecker matrix K
		int n = samplesA.size();
		arma::mat K = Eigen::Matrix<double, 9, 9>::Zero();
		for(int i = 0; i < n; i++){
			const Transform3D& A = samplesA[i];
			const Transform3D& B = samplesB[i];

			K += arma::kron(B.rotation(),A.rotation());

		}

		//Take singular value decomposition of K
		arma::mat U,V;
		Eigen::VectorXd s;
		arma::svd(U,s,V,K);

		// std::cout << "U = \n" << U << std::endl;
		// std::cout << "s = \n" << s << std::endl;
		// std::cout << "V = \n" << V << std::endl;

		//Get index of singular values closest to n
		// Eigen::VectorXd sMinusN = arma::abs(s-double(n));
		// sMinusN.min(index);

		//Use largest singular value
		arma::uword index = 0;

		Eigen::VectorXd u = U.col(index);
		Eigen::VectorXd v = V.col(index);

		// std::cout << "u = \n" << u << std::endl;
		// std::cout << "s(index)  = \n" << s(index) << std::endl;
		// std::cout << "v = \n" << v << std::endl;

		arma::mat V_x = unvectorize(u,3);
		arma::mat V_y = unvectorize(v,3);

		float detV_x = arma::det(V_x);
		float detV_y = arma::det(V_y);

		// std::cout << "det(V_x) = \n" << detV_x << std::endl;
		// std::cout << "det(V_y) = \n" << detV_y << std::endl;

		float alpha_x =  1 / std::cbrt(detV_x);
		float alpha_y =  1 / std::cbrt(detV_y);

		// std::cout << "alpha_x = \n" << alpha_x << std::endl;
		// std::cout << "alpha_y = \n" << alpha_y << std::endl;

		Rotation3D R_y(alpha_x * V_x);
		Rotation3D R_x(alpha_y * V_y);

		// std::cout << "det(R_x) = \n" << arma::det(R_x) << std::endl;
		// std::cout << "det(R_y) = \n" << arma::det(R_y) << std::endl;

		//Translation part

		auto translation = getTranslationComponent(samplesA, samplesB, R_y, success);

		result.first.rotation() = R_x;
		result.second.rotation() = R_y;

		result.first.translation() = translation.first;
		result.second.translation() = translation.second;

		return result;

	}

	std::pair<utility::math::matrix::Transform3D, utility::math::matrix::Transform3D> CalibrationTools::solveClosedForm_Dornaika1998(const std::vector<utility::math::matrix::Transform3D>& samplesA,const std::vector<utility::math::matrix::Transform3D>& samplesB, bool& success){

		std::pair<utility::math::matrix::Transform3D, utility::math::matrix::Transform3D> result;

		//Create kronecker matrix K
		int n = samplesA.size();
		Eigen::Matrix4d C = Eigen::Matrix4d::Zero();
		for(int i = 0; i < n; i++){
			const Transform3D& A = samplesA[i];
			const Transform3D& B = samplesB[i];

			const UnitQuaternion qA(Rotation3D(A.rotation()));
			const UnitQuaternion qB(Rotation3D(B.rotation()));

			Eigen::Matrix4d C_i = -qA.getLeftQuatMultMatrix().t() * qB.getRightQuatMultMatrix();

			C += C_i;

		}

		Eigen::Matrix4d CTC = C.t() * C;

		Eigen::VectorXd eigval;
		arma::mat eigvec;
		arma::eig_sym( eigval, eigvec, CTC );
		std::cout << "eigval = " << eigval << std::endl;
		std::cout << "eigvec = " << eigvec << std::endl;

		Eigen::VectorXd lamda1 = n + Eigen::sqrt(eigval.array()).matrix();
		Eigen::VectorXd lamda2 = n - Eigen::sqrt(eigval.array()).matrix();
		Eigen::VectorXd lamda = arma::join_cols(lamda1,lamda2);
		std::cout << "lamda1 = " << lamda1 << std::endl;
		std::cout << "lamda2 = " << lamda2 << std::endl;
		std::cout << "lamda = " << lamda << std::endl;

		//Find index of smallest non-negative lambda
		float minLamda = std::numeric_limits<float>::max();
		int index = 0;
		for(int i = 0; i < lamda.size(); i++){
			if(lamda[i] < minLamda && lamda[i] >= 0){
				minLamda = lamda[i];
				index = i;
			}
		}
		std::cout << "minLamda = " << minLamda << std::endl;
		std::cout << "index = " << index << std::endl;

		UnitQuaternion qy = eigvec.col(index % lamda1.size());
		std::cout << "qy = " << qy << std::endl;

		UnitQuaternion qx = (1 / (minLamda - n)) * C * qy;
		std::cout << "qx = " << qx << std::endl;

		Rotation3D R_y(qy);
		Rotation3D R_x(qx);

		auto translation = getTranslationComponent(samplesA, samplesB, R_y, success);

		result.first.rotation() = R_x;
		result.second.rotation() = R_y;

		result.first.translation() = translation.first;
		result.second.translation() = translation.second;

		return result;

	}







}
