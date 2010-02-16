#ifndef _Matrix_h_DEFINED
#define _Matrix_h_DEFINED
#include <vector>
#include <math.h>

class Matrix
{

public:

        int		M;			// number of rows
        int		N;			// number of columns
        double*	X;			// matrix WMPointer

        int		getm()	const;		// return number  of rows
        int		getn()	const;		// return number  of columns
        double*	getx()	const;		// return WMPointer to array

        // Constructors

        Matrix();
        Matrix(int m, int n, bool I=false);
        ~Matrix();
        Matrix(const Matrix& a);

        Matrix transp(); // Matrix Transpose
        Matrix getRow(int index); // Get Row
        Matrix getCol(int index); // Get Column

        void setRow(int index, Matrix in); // Set Row
        void setCol(int index, Matrix in); // Set Column

        Matrix&	operator =  (const Matrix& a); // Overloaded Operator

        double*	operator [] (int i)	const; // Overloaded Operator

        void print(); // print values

};

// Overloaded Operators

Matrix	operator +  (const Matrix& a, const Matrix& b);		// Matrix Addition
Matrix	operator -  (const Matrix& a, const Matrix& b);		// Matrix Subtraction
Matrix	operator -  (const Matrix& a, const double& b);		// Matrix Subtraction of a scalar
Matrix	operator *  (const Matrix& a, const Matrix& b);		// Matrix Multiplication
Matrix	operator *  (const double& a, const Matrix& b);		// Matrix multiplication by a scalar
Matrix	operator *  (const Matrix& a, const double& b);		// Matrix multiplication by a scalar
Matrix	operator /  (const Matrix& a, const double& b);		// Matrix division by a scalar

// 2x2 Matrix Inversion

Matrix Invert22(const Matrix& a);

// concatenation
Matrix horzcat(Matrix a, Matrix b);
Matrix vertcat(Matrix a, Matrix b);
Matrix diagcat(Matrix a, Matrix b);

Matrix cholesky(Matrix P);
Matrix HT(Matrix A);
Matrix GaussJordan(Matrix A);       // Returns Gauss Jordan Inverse.  For n x n matrix has n! complexity

inline double convDble(const Matrix& a) { return a[0][0]; } // Convert 1x1 matrix to Double

inline	 int		Matrix::getm() const { return M; }
inline	 int		Matrix::getn() const { return N; }
inline	 double*	Matrix::getx() const { return X; }

#endif

