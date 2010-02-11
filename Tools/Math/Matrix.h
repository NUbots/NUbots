#ifndef _Matrix_h_DEFINED
#define _Matrix_h_DEFINED

#include <vector>
#include <math.h>

class Matrix
{
public:
	int		M;			// number of rows
	int		N;			// number of columns
	double*	X;			// matrix pointer

	int		getm()	const;		// return number  of rows
	int		getn()	const;		// return number  of columns
	double*	getx()	const;		// return pointer to array	 

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
Matrix	operator +  (const Matrix& a, const Matrix& b);
Matrix	operator -  (const Matrix& a, const Matrix& b);
Matrix	operator -  (const Matrix& a, const double& b);
Matrix	operator *  (const Matrix& a, const Matrix& b);
Matrix	operator *  (const double& a, const Matrix& b);
Matrix	operator *  (const Matrix& a, const double& b);
Matrix	operator /  (const Matrix& a, const double& b);


// 2x2 Matrix Inversion
Matrix Invert22(const Matrix& a);
// concatenation
Matrix horzcat(Matrix a, Matrix b);
Matrix vertcat(Matrix a, Matrix b);
Matrix diagcat(Matrix a, Matrix b);
Matrix cholesky(Matrix P);
Matrix HT(Matrix A);

inline double convDble(const Matrix& a) { return a[0][0]; } // Convert 1x1 matrix to Double
inline	 int		Matrix::getm() const { return M; }
inline	 int		Matrix::getn() const { return N; }
inline	 double*	Matrix::getx() const { return X; }

#endif
