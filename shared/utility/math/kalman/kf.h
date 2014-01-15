//TEST FILE!!!!!!!!!!!!!!!!!!!!!!!!
//description of a simplified Kalman filter, not yet overly useful


#ifndef KF_H
#define	KF_H

#include <armadillo>
#include <iostream>

namespace kf { //Kalman Filter
class model {
public:
	arma::mat x;
	arma::mat P;
	arma::mat u;
	arma::mat F;
	arma::mat H;
	arma::mat R;
	arma::mat I;

	model(arma::mat x, arma::mat P, arma::mat u, arma::mat F, arma::mat H, arma::mat R, arma::mat I) : x(x), P(P), u(u), F(F), H(H), R(R), I(I) {}
	friend std::ostream& operator<<(std::ostream& out, const model& m);
}; //class

std::ostream& operator<<(std::ostream& out, const model& m) { //nonmember
	return out << "Kalman Filter Model parameters: " << std::endl <<
	"x" << std::endl << m.x << 
	"P" << std::endl << m.P << 
	"u" << std::endl << m.u << 
	"F" << std::endl << m.F << 
	"H" << std::endl << m.H << 
	"R" << std::endl << m.R << 
	"I" << std::endl << m.I << 
	"--------------------------" << std::endl;
} //nonmemberfunction

void filter(kf::model m, double newmeasure, bool output = false) { // THE ACTUAL FILTER IS STATELESS (thereby, function, not class), refer to cs373, lesson2 for the code

    //measurement update
    arma::mat Z(1, 1); 
    Z << newmeasure;
    arma::mat y = Z - (m.H * m.x);
    arma::mat S = m.H * m.P * m.H.t() + m.R; // t() = transpose
    arma::mat K = m.P * m.H.t() * S.i();     // i() = inverse
    m.x = m.x + (K * y);
    arma::mat P = (m.I - (K * m.H )) * m.P;

    //prediction
    m.x = (m.F * m.x) + m.u; //adding m.u(the motion) to the existing matrix is fine
    m.P = m.F * P * m.F.t();

    //output (debugging)
    if(output) { //test 
            //std::cout << "Z = " << Z << std::endl;
            std::cout << "x = " << std::endl << m.x << std::endl;
            //std::cout << "P = " << std::endl << m.P << std::endl;
    }

} //filter funtion

} //namespace

#endif	/* KF_H */

/* //sample test
 int main() {
	std::cout << "KF test." << std::endl;

	arma::mat x(2, 1); x.zeros();                            // x ~ estimate
	arma::mat P(2, 2); P.eye(); P*=1000.;                    // P ~ uncertianty covariance (decrease uncertianty with measurment, increase uncertianty with movement)
	arma::mat u(2, 1); u.zeros();                            // u ~ motion vector
	arma::mat F(2, 2); F << 1 << 1 << arma::endr << 0 << 1;  // F ~ state transition matrix
	arma::mat H(1, 2); H << 1 << 0;                          // H ~ measurement function
	arma::mat R(1, 1); R.ones();                             // R ~ measurement noise
	arma::mat I(2, 2); I.eye();                              // I ~ identity matrix

	kf::model mymodel(x, P, u, F, H, R, I); // setup & initialise the model (a sample)
	double measurement[] = {1, 2, 3};       // the measurements we put into the filter

	for(int i=0;i < sizeof(measurement)/sizeof(double); i++) {
		kf::filter(mymodel, measurement[i]); // (mymodel, measurement[i], true) for debugging
	}

	std::cout << "End." << std::endl;
	return 0;
}
 
 */