#include "FittingCalculations.h"

FittingCalculations::FittingCalculations()
{
    cx = 0;
    cy = 0;
    r1 = 0;
    r2 = 0;
    theta = 0;
}

void FittingCalculations::SolveEllipse(Array1D<double> a)
{
    theta = atan2(a[1],a[0]-a[2])/2;
    double ct = cos(theta);
    double st = sin(theta);
    double ap = a[0]*ct*ct + a[1]*ct*st + a[2]*st*st;
    double cp = a[0]*st*st - a[1]*ct*st + a[2]*ct*ct;
    Array2D<double> T (2,2,0.0);
    T[0][0] = a[0];
    T[0][1] = a[1]/2;
    T[1][0] = a[1]/2;
    T[1][1] = a[2];
    //cout << "T: " << T << endl;
    Array2D<double> temp (2,1,0.0);
    temp[0][0] = a[3];
    temp[1][0] = a[4]; 
    Array2D<double> twos (2,2,2.0);
    Array2D<double> negOnes(2,2,-1.0);
    Array2D<double> T2 = twos*T;
    Array2D<double> t = matmult(negOnes* Inverse22(T2),temp);

    cx = t[0][0];
    cy = t[1][0];
    //cout << "X: "<<cx << "  Y: " <<cy<< endl;
    //cout << "t: " << t << "T: " <<T<< endl;
    temp = matmult(T,t);
    //cout << temp<<endl;
    Array2D<double> val = matmult(transp(t),temp);
    //cout << val <<endl;
    double scale = 1/(val[0][0] - a[5]);
    r1 = 1/sqrt(scale * ap);
    r2 = 1/sqrt(scale * cp);

    //cout << r1 << endl;
    //cout<< r2 <<endl;
    //cout << theta << endl;
}



Array2D<double> FittingCalculations::Eig(Array2D<double> a)
{
    //cout << "Start EigenVector Decomposition.." << endl; 
    //cout << "M2 Matrix: " << a << endl;

    JAMA::Eigenvalue<double> Eigen(a);
    Array2D<double> evec (a.dim1(),a.dim2(),0.0);
    Array1D<double> evalR (a.dim1(),0.0);
    Eigen.getRealEigenvalues(evalR);
    Eigen.getV(evec);
    //cout << "EigenVectors: " <<evec<< endl;
    //cout << "EigenValues: " <<evalR<< endl;
    return evec;
}

Array2D<double> FittingCalculations::Inverse33(Array2D<double> A)
{
    Array2D<double> X (3,3,0.0);
    Array2D<double> B (3,3,0.0); //the transpose of a matrix A 
    Array2D<double> C (3,3,0.0); //the adjunct matrix of transpose of a matrix A not adjunct of A

     int i,j;
     double x,n=0;//n is the determinant of A
       
     
          for(i=0,j=0;j<3;j++)
          {     
               if(j==2) 
               n+=A[i][j]*A[i+1][0]*A[i+2][1];
               else if(j==1)
               n+=A[i][j]*A[i+1][j+1]*A[i+2][0];
               else 
               n+=A[i][j]*A[i+1][j+1]*A[i+2][j+2];
          }
          for(i=2,j=0;j<3;j++)
          {     
               if(j==2) 
               n-=A[i][j]*A[i-1][0]*A[i-2][1];
               else if(j==1)
               n-=A[i][j]*A[i-1][j+1]*A[i-2][0];
               else
               n-=A[i][j]*A[i-1][j+1]*A[i-2][j+2];
          }


        //A.print();
     
     //cout<<"=====================================================================\n"<<endl;
     //cout<<"The determinant of matrix A is "<<n<<endl<<endl;
     //cout<<"====================================================================="<<endl;
     
     if(n!=0) x=1.0/n;
     else 
     {
          cout<<"Division by 0, not good!\n";
          cout<<"=====================================================================\n"<<endl;
          return X;
     }
     //cout<<"\n========== The transpose of a matrix A ==============================\n";     
     for(i=0;i<3;i++)
     {
          //cout<<endl;
          for(j=0;j<3;j++)
          {     
                    
               B[i][j]=A[j][i];
               //cout<<" B["<<i<<"]["<<j<<"]= "<<B[i][j];
               
          }
     }
     //cout<<endl<<endl;


     C[0][0]=B[1][1]*B[2][2]-(B[2][1]*B[1][2]);
     C[0][1]=(-1)*(B[1][0]*B[2][2]-(B[2][0]*B[1][2]));
     C[0][2]=B[1][0]*B[2][1]-(B[2][0]*B[1][1]);
     
     C[1][0]=(-1)*(B[0][1]*B[2][2]-B[2][1]*B[0][2]);
     C[1][1]=B[0][0]*B[2][2]-B[2][0]*B[0][2];
     C[1][2]=(-1)*(B[0][0]*B[2][1]-B[2][0]*B[0][1]);

     C[2][0]=B[0][1]*B[1][2]-B[1][1]*B[0][2];
     C[2][1]=(-1)*(B[0][0]*B[1][2]-B[1][0]*B[0][2]);
     C[2][2]=B[0][0]*B[1][1]-B[1][0]*B[0][1];


     //cout<<"\n========== The adjunct matrix of transpose of the matrix A ==========\n";     
     //C.print();

     
     for(i=0;i<3;i++)
     {
          for(j=0;j<3;j++)
          {     
               X[i][j]=C[i][j]*x;
               
          }
     }
     //cout<<"\n========== The inverse matrix of the matrix you entered!!! ==========\n";
     //X.print();
    return X;
}

void  FittingCalculations::Print(Array2D<double> a)
{
      int i=0;int j=0;
      for(i=0;i<a.dim1();i++){
              for(j=0;j<a.dim2();j++){
                      std::cout<<a[i][j]<<" ";
                      }
              std::cout<<std::endl;
              }
     std::cout<<std::endl;
    return;
}


Array2D<double> FittingCalculations::transp(Array2D<double> a)
{
	Array2D<double> transpAns(a.dim2(),a.dim1(),0.0);
	int i=0,j=0;
	for (i=0; i<a.dim1(); i++)
	{
		for (j=0; j<a.dim2();j++)
		{
			transpAns[j][i]=a[i][j];
		}
	}
	return transpAns;
}


Array2D<double> FittingCalculations::Inverse22(Array2D<double> a)
{
	Array2D<double> invertAns(a.dim1(),a.dim2());
	invertAns[0][0]=a[1][1];
	invertAns[0][1]=-a[0][1];
	invertAns[1][0]=-a[1][0];
	invertAns[1][1]=a[0][0];
	double divisor=a[0][0]*a[1][1]-a[0][1]*a[1][0];
    Array2D<double> d(invertAns.dim1(),invertAns.dim2(),divisor);
	invertAns=invertAns/d;
	return invertAns;
}
