#include "EllipseFit.h"

#include "EllipseFitting/tnt.h"
#include "EllipseFitting/FittingCalculations.h"
#include <iostream>




EllipseFit::EllipseFit()
{
    //std::cout << "Ellipse Fitting Object Created"<< std::endl;
    cx = 0.0;
    cy = 0.0;
    r1 = 0.0;
    r2 =0.0;
    theta = 0.0;

}

EllipseFit::~EllipseFit()
{
    return;
}

void EllipseFit::Fit_Ellipse(std::vector<LinePoint*> centreCirclePoints)
{
    FittingCalculations fit;
    Array2D<double> x(centreCirclePoints.size(),1,0.0);
    Array2D<double> y(centreCirclePoints.size(),1,0.0);
    for(unsigned int i = 0; i < centreCirclePoints.size(); i++)
    {
            LinePoint* tempPoint = centreCirclePoints.at(i);
            //std::cout << "Point " << i << ": \t"<< tempPoint->x << ",\t"<< tempPoint->y <<std::endl;
            x[i][0] =  tempPoint->x;
            y[i][0] =  tempPoint->y;
    }

    //std::cout<<"Calculating Ellipse Paramters"<< std::endl;

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


    Array2D<double> D1T = fit.transp(D1);
    Array2D<double> D2T = fit.transp(D2);
    Array2D<double> S1 = matmult(D1T,D1);
    Array2D<double> S2 = matmult(D1T,D2);
    Array2D<double> S3 = matmult(D2T,D2);
    Array2D<double> I = fit.Inverse33(S3);
    Array2D<double> S2T = fit.transp(S2);
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
    

    Array2D<double> evec = fit.Eig(M2);

    Array1D<double> cond (evec.dim1(),0.0);
    for(int i =0; i< evec.dim2(); i++)
    {
        cond[i] = 4*evec[0][i]*evec[2][i]-evec[1][i]*evec[1][i];
    } 
    //std::cout << "Conditions: " << cond << std::endl;

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
            //std::cout <<atemp<<std::endl;
            atemp = matmult(T,atemp);
            //std::cout <<"atemp:"<<atemp<<std::endl;

            a1[3]=atemp[0][0];
            a1[4]=atemp[1][0];
            a1[5]=atemp[2][0];

            //std::cout << a1 << std::endl;
        }
        
    }
    fit.SolveEllipse(a1);

    cx = fit.cx;
    cy = fit.cy;
    theta = fit.theta;
    r1 = fit.r1;
    r2 = fit.r2;

    return;
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
void EllipseFit::PrintFinal()
{

    std::cout << "Final Ellipse Fitting Values: "<< std::endl;
    std::cout << "X: " << cx << "\t\t Y: " << cy << std::endl;
    std::cout << "Radius 1: " << r1 << "\t Radius 2:" << r2 << std::endl;
    std::cout << "Theta: " << theta << std::endl;
    return;

}
