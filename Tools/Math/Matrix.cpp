//#include "stdafx.h"
#include "Matrix.h"
#include <string.h>
#include <iostream>

// Constructors
Matrix::Matrix()
{
	M = 0;
	N = 0;
	X = 0;
}

Matrix::Matrix(int m, int n, bool I)//= false)
{
	M=m;
	N=n;
	X=new double [m*n];
	//Initialise matrix to zero
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			(*this)[i][j] = 0;
		}
	}
	//Identity Matrix Initialisation
	if (I)
	{
		if (m != n)
			return;

		for (int i = 0; i < m; i++)
		{
			for (int j = 0; j < n; j++)
			{
				(*this)[i][j] = (i == j) ? 1 : 0;
			}
		}
	}
}

// Copy Constructor
Matrix::Matrix(const Matrix& a)
{
	M=a.M;
	N=a.N;
	X=new double [M*N];
	memcpy(X,a.X,sizeof(double)*M*N);
}

// Destructor
Matrix::~Matrix()
{
    //std::cout << ""
    delete [] X; X = 0;
}

void Matrix::swapRows(unsigned int index1, unsigned int index2)
{
    if(index1 == index2) return;
    Matrix temp = getRow(index1);
    setRow(index1, getRow(index2));
    setRow(index2, temp);
    return;
}

// Matrix Index Operator
// Returns a pointer to the ith row in the matrix


// Matrix Addition
Matrix operator + (const Matrix& a, const Matrix& b)
{
	Matrix addAns(a.getm(),a.getn());
	int i=0,j=0;
	if ((a.getn()==b.getn())&&(a.getm()==b.getm()))
	{
		for (i=0; i<a.getm(); i++)
		{
			for (j=0; j<a.getn();j++)
			{
				addAns[i][j]=a[i][j]+b[i][j];
			}
		}
	}
	return addAns;
	//This return calls the copy constructor which copies the matrix into another block of memory
	//and then returns the pointer to this new memory.
	//Otherwise the array addAns is deleted by the destructor here and the pointer returned from the addition
	//is a pointer to deleted memory. This causes problems when the function calling this tries to delete this memory again.
}

// Matrix Subtraction
Matrix	operator -  (const Matrix& a, const Matrix& b)
{
	Matrix subAns(a.getm(),a.getn());
	int i=0,j=0;
	if ((a.getn()==b.getn())&&(a.getm()==b.getm()))
	{
		for (i=0; i<a.getm(); i++)
		{
			for (j=0; j<a.getn();j++)
			{
				subAns[i][j]=a[i][j]-b[i][j];
			}
		}
	}
	return subAns;
}

// Matrix Subtraction
Matrix	operator -  (const Matrix& a, const double& b)
{
	Matrix subAns(a.getm(),a.getn());
	int i=0,j=0;
	for (i=0; i<a.getm(); i++)
	{
		for (j=0; j<a.getn();j++)
		{
			subAns[i][j]=a[i][j]-b;
		}
	}

	return subAns;
}

//Matrix Multiplication
Matrix	operator * (const Matrix& a, const Matrix& b)
{
	Matrix multAns(a.getm(),b.getn());
	int i=0,j=0,k=0;
	if (a.getn()==b.getm())
	{
		for (i=0; i<a.getm(); i++)
		{
			for (j=0; j<b.getn();j++)
			{
				double temp=0;
				for (k=0; k<a.getn(); k++)
				{
                    temp+=a[i][k]*b[k][j];
				}
				multAns[i][j]=temp;
			}
		}
	}
	else if (a.getm()==b.getm() && a.getn()==b.getn())
	{
		for (i=0; i<a.getm(); i++)
		{
			for (j=0; j<b.getn();j++)
			{
				multAns[i][j]=a[i][j]*b[i][j];
			}
		}
	}
	return multAns;
}

// Matrix Multiplication by a Scalar
Matrix	operator * (const double& a, const Matrix& b)
{
	Matrix multAns(b.getm(),b.getn());
	int i=0,j=0;
	for (i=0; i<b.getm(); i++)
	{
		for (j=0; j<b.getn();j++)
		{
			multAns[i][j]=b[i][j]*a;
		}
	}
	return multAns;
}

// Matrix Multiplication by a Scalar
Matrix	operator * (const Matrix& a, const double& b)
{
	Matrix multAns(a.getm(),a.getn());
	int i=0,j=0;
	for (i=0; i<a.getm(); i++)
	{
		for (j=0; j<a.getn();j++)
		{
			multAns[i][j]=a[i][j]*b;
		}
	}
	return multAns;
}

// Matrix Division by a Scalar
Matrix	operator / (const Matrix& a, const double& b)
{
	Matrix divAns(a.getm(),a.getn());
	int i=0,j=0;
	for (i=0; i<a.getm(); i++)
	{
		for (j=0; j<a.getn();j++)
		{
			divAns[i][j]=a[i][j]/b;
		}
	}
	return divAns;
}

// Matrix Equality
Matrix& Matrix::operator =  (const Matrix& a)
{
	if (X!=0)
		delete [] X;
	M=a.M;
	N=a.N;
	X=new double [M*N];
	memcpy(X,a.X,sizeof(double)*M*N);
	return *this;
}

// Matrix Transpose
Matrix	Matrix::transp() const
{
	Matrix transpAns(getn(),getm());
	int i=0,j=0;
	for (i=0; i<getm(); i++)
	{
		for (j=0; j<getn();j++)
		{
			transpAns[j][i]=(*this)[i][j];
		}
	}
	return transpAns;
}

Matrix  Matrix::getRow(int index) const
{
    Matrix Row (1,getn());
    int i=0;
    for(i=0; i<getn();i++){
            Row[0][i]=(*this)[index][i];
            }
    return Row;
}

Matrix  Matrix::getCol(int index) const
{
    Matrix Col (getm(),1);
    int i=0;
    for(i=0; i<getm();i++){
            Col[i][0]=(*this)[i][index];
            }
    return Col;
}

void  Matrix::setRow(int index, Matrix in)
{
    int i=0;
    for(i=0; i<getn();i++){
            (*this)[index][i]=in[0][i];
            }
}

void  Matrix::setCol(int index, Matrix in)
{
    int i=0;
    for(i=0; i<getm();i++){
            (*this)[i][index]=in[i][0];
            }
}

void  Matrix::print()
{
      int i=0;int j=0;
      for(i=0;i<(*this).getm();i++){
              for(j=0;j<(*this).getn();j++){
                      std::cout<<(*this)[i][j]<<" ";
                      }
              std::cout<<std::endl;
              }
     std::cout<<std::endl;
}

std::vector<float> Matrix::asVector()
{
    int rows, cols;
    rows = getm();
    cols = getn();
    int length = rows * cols;
    std::vector<float> result(length);
    int row = 0, col = 0;
    for(int i = 0; i < length; i++)
    {
        row = i / cols;
        col = i % cols;
        result[i] = (*this)[row][col];
    }
    return result;
}

bool Matrix::isValid() const
{
    if((*this).getm() == 0 or (*this).getn() == 0)
        return false;
    for(int i=0;i<(*this).getm();i++)
    {
        for(int j=0;j<(*this).getn();j++)
        {
            if( (*this)[i][j] != (*this)[i][j] )
            {
                return false;
            }
        }
    }
    return true;
}

bool Matrix::operator ==(const Matrix& b) const
{
    float local_m = getm();
    float local_n = getn();

    // first compare dimensions
    if(local_m != b.getm()) return false;
    if(local_n != b.getn()) return false;

    // now compare each value.
    for(unsigned int i=0;i<local_m;++i)
    {
        for(unsigned int j=0;j<local_n;j++)
        {
            if( (*this)[i][j] != b[i][j])
            {
                return false;
            }
        }
    }
    return true;
}

Matrix Matrix4x4fromVector(const std::vector<float>& source)
{
    Matrix result(4,4,false);
    if(source.size() != 16) return result;
    int row, col, cols;
    cols = result.getn();
    for(int i = 0; i < 16; i++)
    {
        row = i / cols;
        col = i % cols;
        result[row][col] = source[i];
    }
    return result;
}

// 2x2 Matrix Inversion- t
Matrix Invert22(const Matrix& a)
{
	Matrix invertAns(a.getm(),a.getn());
	invertAns[0][0]=a[1][1];
	invertAns[0][1]=-a[0][1];
	invertAns[1][0]=-a[1][0];
	invertAns[1][1]=a[0][0];
	double divisor=a[0][0]*a[1][1]-a[0][1]*a[1][0];
	invertAns=invertAns/divisor;
	return invertAns;
}
Matrix vertcat(Matrix a, Matrix b)
{
    //Matrix concatenation
    //assume same dimension on cols
    int mTotal=a.getm()+b.getm();
    Matrix c=Matrix(mTotal,a.getn(),false);
    for(int i=0;i<a.getm();i++){
            c.setRow(i,a.getRow(i));
            }
    for(int i=0;i<b.getm();i++){
            c.setRow(i+a.getm(),b.getRow(i));
            }
    return c;
}

Matrix horzcat(Matrix a, Matrix b)
{
    //Matrix concatenation
    //assume same dimension on rows
    int nTotal=a.getn()+b.getn();
    Matrix c=Matrix(a.getm(),nTotal,false);
    for(int i=0;i<a.getn();i++){
            c.setCol(i,a.getCol(i));
            }
    for(int i=0;i<b.getn();i++){
            c.setCol(i+a.getn(),b.getCol(i));
            }
    return c;
}
Matrix diagcat(Matrix a, Matrix b)
{
    //Matrix concatenation
    //assume both Matrix a and b are square
    int mTotal=a.getm()+b.getm();
    int nTotal=a.getn()+b.getn();
    Matrix c=Matrix(mTotal,nTotal,false);
    for(int i=0;i<a.getm();i++){
            for(int j=0;j<a.getn();j++){
                c[i][j]=a[i][j];
            }
    }
    for(int i=0;i<b.getm();i++){
        for(int j=0;j<b.getn();j++){
                c[i+a.getm()][j+a.getn()]=b[i][j];
            }
    }
    return c;
}
Matrix cholesky(Matrix P)
{
    Matrix L = Matrix (P.getm(),P.getn(),false);
    double a=0;
    for(int i=0;i<P.getm();i++)
    {
        for(int j=0;j<i;j++)
        {
			a=P[i][j];
            for(int k=0;k<j;k++)
            {
                a=a-L[i][k]*L[j][k];
            }
            L[i][j]=a/L[j][j];
		}
		a=P[i][i];
        for(int k=0;k<i;k++)
        {
			a=a-pow(L[i][k],2);
		}
		L[i][i]=sqrt(a);
	}
    return L;
}

Matrix HT(Matrix A)
{
    //Householder Triangularization Algorithm
    //rows = n in the algorithm, for avoid confusion.
	int rows=A.getm();
	int r=A.getn()-rows;
	double sigma;
	double a;
	double b;
	std::vector <double> v (A.getn());
	Matrix B= Matrix(rows,rows,false);
	for(int k=rows-1;k>=0;k--){
		sigma=0.0;
		for(int j=0;j<=r+k;j++){
			sigma=sigma+A[k][j]*A[k][j];
		}
		a=sqrt(sigma);
		sigma=0.0;
		for(int j=0;j<=r+k;j++){
			if(j==r+k){
				v.at(j)=(A[k][j]-a);
			}
			else{
				v.at(j)=(A[k][j]);
			}
			sigma=sigma+v.at(j)*v.at(j);
		}
		a=2.0/(sigma+1e-15);
		for(int i=0;i<=k;i++){
			sigma=0.0;
			for(int j=0;j<=r+k;j++){
				sigma=sigma+A[i][j]*v.at(j);
			}
			b=a*sigma;
			for(int j=0;j<=r+k;j++){
				A[i][j]=A[i][j]-b*v.at(j);
			}
		}
	}
	for(int i=0;i<rows;i++){
		B.setCol(i,A.getCol(r+i));
	}
	return B;
}


double determinant(const Matrix& mat)
{
    if(mat.getm()==1)
        return mat[0][0];
    else if(mat.getm()==2)
        return (mat[0][0]*mat[1][1]-mat[0][1]*mat[1][0]);                                              
    
    double det = 0;
    
    Matrix * subMat[mat.getm()]; 
    for(int i=0; i<mat.getm(); i++)
    {
       subMat[i] = new Matrix(mat.getm()-1, mat.getm()-1); 
    }
    
    for(int i=0; i<mat.getm(); i++)
    {
        for(int j=0; j<mat.getm()-1; j++)
        {
            for(int k=0, l=0; k<mat.getm()-1; k++,l++)
            {
                if(i==l)
                    l++; 
                (*(subMat[i]))[j][k]=mat[j+1][l];
            }
        }   
    }
    
    for(int i=0, sign=1; i<mat.getm(); i++)
    {
        det+=sign*mat[0][i]*determinant(*subMat[i]);
        if(sign==-1)
            sign=1;
        else
            sign=-1;
    }
    
    for(int i=0; i<mat.getm(); i++)
    {
       delete subMat[i];
    }
    
    return det;
}

Matrix CofactorMatrix(const Matrix& mat)
{
    Matrix coMat(mat.getm(),mat.getn());
    for(int i=0; i<mat.getm(); i++)
    {
        for(int j=0; j<mat.getn(); j++)
        {
            Matrix * minMat = new Matrix(mat.getm()-1, mat.getm()-1);
            for(int k=0, l=0; k<mat.getn()-1; k++, l++)
            {
                if(k==i)
                    l++; 
                for(int m=0, n=0; m<mat.getn()-1; m++, n++)
                {
                    if(m==j)
                        n++;
                    (*minMat)[k][m]=mat[l][n];
                }
            }
            if((i+j)%2==0)
                 coMat[i][j]=determinant(*minMat);
            else
                 coMat[i][j]=-determinant(*minMat);
            delete minMat;        
        }        
    }  
    return coMat;      
}

Matrix InverseMatrix(const Matrix& mat)
{
    if(mat.getm() == 1)
    {
        Matrix result(1,1,false);
        result[0][0] = 1.f / mat[0][0];
        return result;
    }
    else if(mat.getm() == 2)
        return Invert22(mat);
    else
        return GaussJordanInverse(mat);
}

ostream& operator <<(ostream& out, const Matrix &mat)
{
    for(int i=0; i<mat.getm(); i++)
    {
        out << "[ ";
        for(int j=0; j<mat.getn(); j++)
        {
            out << std::setw(12) << std::setprecision(4) << mat[i][j];
        }
        out << "]\n";
    }
    return out;        
}

double dot(const Matrix& mat1, const Matrix& mat2)
{
     double ret = 0;
     for(int i = 0; i<mat1.getm(); i++)
         ret+=mat1[i][0]*mat2[i][0]; 
     return ret;        
          
}

Matrix CramersRuleInverse(const Matrix& mat)
{
    return (CofactorMatrix(mat)).transp()/determinant(mat);
}

Matrix GaussJordanInverse(const Matrix& mat)
{
    // Augment matrix with I - [mat | I]
    Matrix A = horzcat(mat, Matrix(mat.getm(), mat.getn(), true));

    int i,j;
    unsigned int i_max = 0;
    int k = 0;
    double C;
    for (k = 0; k < A.getm(); ++k)
    {
        // find max pivot.
        i_max = k;
        for (i = k; i < A.getm(); ++i)
        {
            if(fabs(A[i_max][k]) < fabs(A[i][k]))
                i_max = i;
        }

        // Check if singular
        if(A[i_max][k] == 0)
            std::cout << "Matrix is singular" << std::endl;

        A.swapRows(k, i_max);
        for(i = k+1; i < A.getm(); ++i)
        {
            C = A[i][k] / A[k][k];
            for(j = k+1; j < A.getn(); ++j)
            {
                A[i][j] -= A[k][j] * C;
            }
            A[i][k] = 0;
        }
    }
    // Back substitution
    for(k = A.getm()-1; k >= 0; --k)
    {
        C = A[k][k];
        for(i=0; i < k; ++i)
        {
            for(j=A.getn()-1; j > k-1; --j)
            {
                A[i][j] -= A[k][j] * A[i][k] / C;
            }
        }
        A.setRow(k, A.getRow(k) / C);
    }

    // Un-Augment matrix from I - [I | result]
    Matrix result(mat.getm(), mat.getn(), false);
    for(i=0; i < mat.getn(); ++i)
        result.setCol(i, A.getCol(i + mat.getn()));
    return result;
}

Matrix CholeskyUpdate(Matrix S, Matrix U, float v)
{
    const unsigned int size = U.getm();
    const unsigned int cols = U.getn();
    const int sign = v >= 0.f ? 1 : -1;
    // Do each of the collumns.
    U = sqrt(fabs(v)) * U;
    for(unsigned int col = 0; col < cols; ++col)
    {
        for(unsigned int k = 0; k < size; ++k)
        {
            const double r = sqrt(pow(S[k][k],2) + sign * pow(U[k][col],2));
            const double c = r / S[k][k];
            const double s = U[k][col] / S[k][k];
            S[k][k] = r;
            for(unsigned int j = k+1; j < size; ++j)
            {
                S[k][j] = (S[k][j] + sign * s * U[j][col]) / c;
                U[j][col] = c * U[j][col] - s * S[k][j];
            }
        }
    }
    return S;
}

Matrix QR_Householder(const Matrix& A)
{
    Matrix input = A.getm() >= A.getn() ? A : A.transp();
    const unsigned int rows = input.getm();
    const unsigned int cols = input.getn();
    const unsigned int size = std::min(rows-1, cols);

    Matrix V(rows, 1, false);
    double a_norm, temp, beta;

    for(unsigned int k = 0; k < size; ++k)
    {
        // Equivalent of HOUSEHOLDER function.
        temp = 0.0;
        for(unsigned int i = k; i < rows; ++i)
        {
            temp += input[i][k] * input[i][k];
        }
        a_norm = sqrt(temp);
        beta = input[k][k] >= 0 ? input[k][k] + a_norm :  input[k][k] - a_norm;
        V[k][0] = 1.0;
        for(unsigned int i = k+1; i < rows; ++i)
        {
            V[i][0] = 1 / beta * input[i][k];
        }

        // Equivalent of HOUSEHOLDER_MULT function
        // 	A(k:n,k:m) = HOUSEHOLDER_MULT(A(k:n,k:m),v(k:n,1));
        double vv = 0.0;
        for(unsigned int i = k; i < rows; ++i)
        {
            vv += V[i][0] * V[i][0];
        }
        beta = -2 / vv;
        Matrix w(1, rows, false);
        for(unsigned int col = k; col < cols; ++col)
        {
            for(unsigned int row = k; row < rows; ++row)
            {
                w[0][col] += V[row][0] * input[row][col];
            }
        }

        for(unsigned int col = k; col < cols; ++col)
        {
            for(unsigned int row = k; row < rows; ++row)
            {
                input[row][col] = input[row][col] + beta * V[row][0] * w[0][col];
            }
        }

        for(unsigned int row = k+1; row < rows; ++row)
        {
            input[row][k] = V[row][0];
        }
    }
    Matrix result(size,size, false);
    for(unsigned int k = 0; k < size; ++k)
    {
        for(unsigned int l = k; l < size; ++l)
        {
            result[k][l] = input[k][l];
        }
    }

    return result;
}

Matrix diag(const Matrix& A)
{
    Matrix result;
    if(A.getm() == 1)
    {
        unsigned int size = A.getn();
        result = Matrix(size, size, false);
        for(unsigned int i = 0; i < size; ++i)
        {
            result[i][i] = A[0][i];
        }
    }
    else if(A.getn() == 1)
    {
        unsigned int size = A.getn();
        result = Matrix(size, size, false);
        for(unsigned int i = 0; i < size; ++i)
        {
            result[i][i] = A[i][0];
        }
    }
    return result;
}

void WriteMatrix(std::ostream& out, const Matrix &mat)
{
    int m = mat.getm(), n = mat.getn();
    double element;
    out.write(reinterpret_cast<const char*>(&m),sizeof(m));
    out.write(reinterpret_cast<const char*>(&n),sizeof(n));
    for(int i=0; i<m; i++)
    {
        for(int j=0; j<n; j++)
        {
            element = mat[i][j];
            out.write(reinterpret_cast<const char*>(&element),sizeof(element));
        }
    }
    return;
}

Matrix ReadMatrix(std::istream& in)
{
    int m, n;
    double element;
    in.read(reinterpret_cast<char*>(&m),sizeof(m));
    in.read(reinterpret_cast<char*>(&n),sizeof(n));
    if(m == 0 or n == 0)
        return Matrix();
    Matrix result(m,n,false);
    for(int i=0; i<m; i++)
    {
        for(int j=0; j<n; j++)
        {
            in.read(reinterpret_cast<char*>(&element),sizeof(element));
            result[i][j] = element;
        }
    }
    return result;
}

