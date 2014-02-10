//the following function takes in the 4 normal force vectors from a foot
//and tries to generate a 3d vector that points to the center of mass.
//naturally, this seems a bit fishy (particularly in the scaling of the z-component)
//but may prove useful in future
//Author: Andrew Dabson, 2014

//F1,F2,F3,F4 are the forces on each resistor
//
// F4/-----\F1  *
//   |.x   |    |                *
//   | .   |    | FOOTLENGTH     | LENGTHOFFSET
//   | .   |    |                |
// F3\_.___/F2  *                *
//
//   *-----* FOOTWIDTH
//   *-* WIDTHOFFSET

#define FOOTWIDTH    300 //milimeter distance from fsr's on the left side on each foot to fsrs on the right side of each foot
#define FOOTLENGTH   600 //(sample) milimeter distance from heel fsr's to to fsr's
#define WIDTHOFFSET  150 //measured from the left edge of the foot (F3-F4), where is our origin
#define LENGTHOFFSET 300 //measured from the heel of the foot (F3-f2)

#ifndef UTILITY_MATH_FOOTNORMAL
#define UTILITY_MATH_FOOTNORMAL
#include <iostream>
#include <armadillo>

namespace utility {
	namespace math {

		//int main() { //testing
			//std::cout << "output: " << std::endl;
			//std::cout << footnormal(3.1, 2.7, 1.4, 0.99) << std::endl << std::endl;
			//std::cout << footnormal(1,1,1,1) << std::endl << std::endl;
			//std::cout << footnormal(2,2,1,1) << std::endl << std::endl; //should lean to the positive x
			//std::cout << footnormal(2,1,2,1) << std::endl << std::endl; //should be perpendicular
			//std::cout << footnormal(1,2,2,1) << std::endl << std::endl; //should lean to the negative y
			//std::cout << footnormal(2,1,1,1) << std::endl << std::endl; //should lean to the positve xy

			//std::cout << footnormal(1,0,0,0) << std::endl << std::endl; //AHHH, this is where its faling apart. It should be pointing relative to width and height (currently 1,1,1)
			//return 0;
		//}

		//function footvector() infers a normal vector based on the 4 fsr's (force sensitive resistors) in each foot
		//input:  4 forces(Newtons) correspoonding to each foot sensor
		//output: a vector pointing to the center of mass
		arma::vec footnormal(double f1, double f2, double f3, double f4) {

			//create the vectors FROM origin TO FSR's
			arma::mat q(4,3);
			q(0,0) = FOOTWIDTH - WIDTHOFFSET;
			q(0,1) = FOOTLENGTH - LENGTHOFFSET;

			q(1,0) = FOOTWIDTH - WIDTHOFFSET;
			q(1,1) = -LENGTHOFFSET;

			q(2,0) = -WIDTHOFFSET;
			q(2,1) = -LENGTHOFFSET;

			q(3,0) = -WIDTHOFFSET;
			q(3,1) = FOOTLENGTH - LENGTHOFFSET;

			q(0,2) = 1;
			q(1,2) = 1;
			q(2,2) = 1;
			q(3,2) = 1;

			//normalise the forces as (percentage) weights
			double sum = f1 + f2 + f3 + f4; //z coord & sum of feet

			arma::vec weights;
			weights << f1 << f2 << f3 << f4;
			weights /= sum; //normalise 

			//generate which direction the normal vector should be pointing
			arma::vec v; //v is a 3d vector
			v.zeros(3);

			v = (weights.t() * q).t(); //as expected, the z-components are always 1.
			v[2] = sum; //set the z-component equal to the sum of the 4 forces

			return v;
		} //end function
	} //close namespace math
} //close namespace utility
#endif