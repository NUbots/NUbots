#include "EllipseFit.h"

#include <iostream>

using namespace std;


EllipseFit::EllipseFit()
{
    //cout << "Ellipse Fitting Object Created"<< endl;
    cx = 0.0;
    cy = 0.0;
    r1 = 0.0;
    r2 =0.0;
    theta = 0.0;
    return;
}

EllipseFit::~EllipseFit()
{
    return;
}

void EllipseFit::Fit_Ellipse(Array2D<double> x, Array2D<double> y)
{
   
   

    //cout<<"Calculating Ellipse Paramters"<< endl;

    Array2D<double> X2 = x*x;
    Array2D<double> XY = x*y;
    Array2D<double> Y2 = y*y;


    Array2D<double> D1(x.dim1(),3,0.0);
    Array2D<double> D2(x.dim1(),3,0.0);
    Array2D<double> ones(x.dim1(),3,1.0);
    for (int i = 0; i < x.dim1();i++)
    {
        D1[i][0] = X2[i][0];
        D1[i][1] = XY[i][0];
        D1[i][2] = Y2[i][0];
        D2[i][0] = x[i][0];
        D2[i][1] = y[i][0];
        D2[i][2] = ones[i][0];
    }


    Array2D<double> D1T = transp(D1);
    Array2D<double> D2T = transp(D2);
    Array2D<double> S1 = matmult(D1T,D1);
    Array2D<double> S2 = matmult(D1T,D2);
    Array2D<double> S3 = matmult(D2T,D2);
    Array2D<double> I = Inverse33(S3);
    Array2D<double> S2T = transp(S2);
    Array2D<double> NegOnes(3,3,-1);
    Array2D<double> T = matmult((NegOnes * I),S2T);;
    Array2D<double> M = matmult(S2,T);
    M = M + S1;
    Array2D<double> M2 (M.dim1(),M.dim2(),0.0);
    for(int i = 0; i < M.dim2(); i++)
    {
        M2[0][i] = M[2][i]/2.0;
        M2[1][i] = -1.0 * M[1][i];
        M2[2][i] = M[0][i]/2.0; 
    }
    

    Array2D<double> evec = Eig(M2);

    Array1D<double> cond (evec.dim1(),0.0);
    for(int i =0; i< evec.dim2(); i++)
    {
        cond[i] = 4*evec[0][i]*evec[2][i]-evec[1][i]*evec[1][i];
    } 
    //cout << "Conditions: " << cond << endl;

    //Find cond>  0
    
    Array1D<double> a1 (evec.dim1()*2,0.0);
    for (int i =0; i < cond.dim1(); i++)
    {
        if(cond[i] > 0)
        {
            a1[0]=evec[0][i];
            a1[1]=evec[1][i];
            a1[2]=evec[2][i];
         
            Array2D<double> atemp (evec.dim1(),1,0.0);
            atemp[0][0]=evec[0][i];
            atemp[1][0]=evec[1][i];
            atemp[2][0]=evec[2][i];
            //T * a1
            //cout <<atemp<<endl;
            atemp = matmult(T,atemp);
            //cout <<"atemp:"<<atemp<<endl;

            a1[3]=atemp[0][0];
            a1[4]=atemp[1][0];
            a1[5]=atemp[2][0];

            //cout << a1 << endl;
        }
        
    }
    SolveEllipse(a1);
    return;
}

void EllipseFit::SolveEllipse(Array1D<double> a)
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



Array2D<double> EllipseFit::Eig(Array2D<double> a)
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

Array2D<double> EllipseFit::Inverse33(Array2D<double> A)
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

void  EllipseFit::Print(Array2D<double> a)
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

void EllipseFit::PrintFinal()
{
    
    cout << "Final Ellipse Fitting Values: "<< endl;
    cout << "X: " << cx << "\t\t Y: " << cy << endl;
    cout << "Radius 1: " << r1 << "\t Radius 2:" << r2 << endl;
    cout << "Theta: " << theta << endl;
    return;

}

Array2D<double> EllipseFit::transp(Array2D<double> a)
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


Array2D<double> EllipseFit::Inverse22(Array2D<double> a)
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

double EllipseFit::GetX()
{
    return cx;
}

double EllipseFit::GetY()
{
    return cy;
}
double EllipseFit::GetR1()
{
    return r1;
}

double EllipseFit::GetR2()
{
    return r2;
}

double EllipseFit::GetTheta()
{
    return theta;
}
