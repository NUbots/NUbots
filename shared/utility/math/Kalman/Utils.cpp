#include "Utils.h"
namespace duti{
    /*/this function should be unneccesary now (though its still more efficient than the armadillo way of doing it: A==B ..in arma is (arma::all(ARMA::VECTORISE(A==B)))
bool equals(const arma::mat a, const arma::mat b) { 
    if(a.n_rows != b.n_rows)
        return false;
    
    if(a.n_cols != b.n_cols)
        return false;
    
    for(int i=0;i<a.n_rows;i++) {//using the built in A == B needlessly generates a new matrix.
        for(int j=0;j<a.n_cols;j++) {
            if(a(i, j) != b(i, j)){ //BE WARY WITH FLOAT COMPARRISON!!! -
                return false;
            }//if
        }//inner for
    }//outer for
    return true;
}
*/
    
//duti::isMatrixValid
bool isMatrixValid(arma::mat m) {    //used in MultivariateGaussian.cpp and Kalman.cpp
    //this may be useless

    if(m.n_rows == 0 || m.n_cols == 0)
        return false;
}

} //namespace