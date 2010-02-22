//#include "stdafx.h"
#include "Matrix.h"
#include <string.h>
#include <iostream>
#include <iomanip>

// Constructors
Matrix::Matrix()
{
        M = 0;
        N = 0;
        X = 0;
}

Matrix::Matrix(int m, int n, bool I/*= false*/)
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
        delete [] X; X = 0;
}

// Matrix Index Operator
// Returns a WMPointer to the ith row in the matrix
double*	Matrix::operator []	(int i)	const
{ return &X[i*N]; }

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
        //and then returns the WMPointer to this new memory.
        //Otherwise the array addAns is deleted by the destructor here and the WMPointer returned from the addition
        //is a WMPointer to deleted memory. This causes problems when the function calling this tries to delete this memory again.
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

Matrix	Matrix::transp()

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

Matrix  Matrix::getRow(int index)
{
    Matrix Row (1,getn());
    int i=0;
    for(i=0; i<getn();i++){
            Row[0][i]=(*this)[index][i];
            }
    return Row;
}

Matrix  Matrix::getCol(int index)
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
    for(i=0; i<getn();i++)
        {
        (*this)[index][i]=in[0][i];
    }
}

void  Matrix::setCol(int index, Matrix in)
{
    int i=0;
    for(i=0; i<getm();i++)
        {
        (*this)[i][index]=in[i][0];
    }
}

void  Matrix::print()
{
      int i=0;int j=0;
      for(i=0;i<(*this).getm();i++)
          {
              for(j=0;j<(*this).getn();j++)
                          {
                      std::cout<<std::fixed<<std::setprecision(3)<<(*this)[i][j]<<"\t";
              }
              std::cout<<std::endl;
      }
     std::cout<<std::endl;
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
    for(int i=0;i<a.getm();i++)
        {
                c.setRow(i,a.getRow(i));
    }
    for(int i=0;i<b.getm();i++)
        {
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
    for(int i=0;i<a.getn();i++)
        {
        c.setCol(i,a.getCol(i));
    }

    for(int i=0;i<b.getn();i++)
        {
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

    for(int i=0;i<a.getm();i++)
        {
            for(int j=0;j<a.getn();j++)
                        {
                c[i][j]=a[i][j];
            }
    }
    for(int i=0;i<b.getm();i++)
        {
        for(int j=0;j<b.getn();j++)
                {
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

        for(int k=rows-1;k>=0;k--)
        {
                sigma=0.0;
                for(int j=0;j<=r+k;j++)
                {
                        sigma=sigma+A[k][j]*A[k][j];
                }
                a=sqrt(sigma);
                sigma=0.0;
                for(int j=0;j<=r+k;j++)
                {
                        if(j==r+k)
                        {
                                v.at(j)=(A[k][j]-a);
                        }
                        else
                        {
                                v.at(j)=(A[k][j]);
                        }
                        sigma=sigma+v.at(j)*v.at(j);
                }
                a=2.0/(sigma+1e-15);
                for(int i=0;i<=k;i++)
                {
                        sigma=0.0;
                        for(int j=0;j<=r+k;j++)
                        {
                                sigma=sigma+A[i][j]*v.at(j);
                        }
                        b=a*sigma;
                        for(int j=0;j<=r+k;j++)
                        {
                                A[i][j]=A[i][j]-b*v.at(j);
                        }
                }
        }
        for(int i=0;i<rows;i++)
        {
                B.setCol(i,A.getCol(r+i));
        }
        return B;
}

Matrix GaussJordan(Matrix A)
{
        if (A.getm() != A.getn())
            return A;

        Matrix Left = A;                                //Create Matrix to reduce
        Matrix Right = Matrix(A.getm(),A.getn(),true);  //Create Identity matrix to transform into inverse

        //Reduction
        for (int i = 0;i<A.getn();i++)
        {
            // Divide current row by leading digit to get leading 1
            double divisor = Left[i][i];

            //If leading digit 0 (or negligably close), add next non-zero row leading
            if (divisor <= 0.0000001 && divisor >= -0.0000001)
            {

                int non_zero = i;
                double x = 0;
                //Check for next non-zero leading
                while (x > -0.0000001 && x < 0.0000001)
                {
                    non_zero++;
                    x = Left[non_zero][i];
                    //If no more non zero, matrix is not invertible so return original
                    if(x <= 0.0000001 && x >= -0.0000001 && non_zero == A.getn() - 1)
                        return A;
                }

                //Add row
                for (int i2 = A.getn()-1; i2 >=0; i2--)
                {
                    Right[i][i2] = (Right[i][i2] + Right[non_zero][i2]);
                    Left[i][i2] = (Left[i][i2] + Left[non_zero][i2]);
                }
                divisor = Left[i][i];
            }


            for (int j = A.getn()-1; j >=0 ;j--)
            {
                Right[i][j] = (Right[i][j] / divisor);
                Left[i][j] = (Left[i][j] / divisor);
            }
            // Subtract each consecutive row by multiples of previous row to get leading 0's
            for (int k = i+1;k<A.getn();k++)
            {
                if (Left[k][i]!=0)   //If not already leading zero
                {
                    double leading = Left[k][i];
                    for (int a = A.getn()-1;a>=0;a--)
                    {
                         Right[k][a] = (Right[k][a] - leading*Right[i][a]);
                         Left[k][a] = (Left[k][a] - leading*Left[i][a]);
                    }
                }
            }
        }
        //Back substitution
        //Working backwards from the second last row, divide each column by a multiple of the previous row to get trailing 0's

        for (int i = A.getn()-2;i>=0;i--)
        {
            for(int j = A.getn()-1;j>i;j--)
            {
                double backTemp = Left[i][j];
                for(int k = A.getn()-1;k >=0;k--)
                {
                    Right[i][k] = (Right[i][k] - backTemp *Right[j][k]);
                    Left[i][k] = (Left[i][k] - backTemp *Left[j][k]);
                }
            }
        }

        return Right;
}

