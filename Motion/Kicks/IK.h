#ifndef H_IK
#define H_IK

#include "Tools/Math/Matrix.h"
#include "Tools/Math/General.h"
#include <cstdlib>
#include <vector>
using namespace mathGeneral;

//#define PI 3.141592653589793
#define SQRT_TWO 1.414213562
#define SQRT_HLF 0.707106781

//NaoParams
const double HY = 5;
const double HZ = 8.5;
const double FH = 4.6;
const double FL = 9.5;
const double ThL = 10;
const double TiL = 10;
    
const double alphArrL[] = {-3*PI/4,-PI/2,PI/2,0,0,-PI/2};
const double aArrL[] = {0,0,0,-ThL,-TiL,0};
const double thArrL[] = {-PI/2,PI/4,0,0,0,0};
const double dArrL[] = {0,0,0,0,0,0};

const double alphArrR[] = {-PI/4,-PI/2,PI/2,0,0,-PI/2};
const double aArrR[] = {0,0,0,-ThL,-TiL,0};
const double thArrR[] = {-PI/2,-PI/4,0,0,0,0};
const double dArrR[] = {0,0,0,0,0,0};
//endNaoParams

using std::vector;
enum legChoice {NONE, LEFT, RIGHT}; 

class Joint
{
public:
    Joint();
    Joint(const Joint& j);
    Joint(double alpha, double a, double theta, double d);
    ~Joint();
	Matrix * getTransformMatrix();
	Matrix * getDiffTransformMatrix();
    void updateTransforms();
    double& getTheta(){return theta;};
    Joint& operator=(const Joint& j);
private:
    Matrix createTransformMatrix();
    Matrix createDiffTransformMatrix(); 
    double alpha;
    double a;
    double theta;
    double d;
    Matrix trans;
    Matrix diff;
};

class JointSystem
{
public:
    JointSystem();
    ~JointSystem();
    void addJoint(Joint &j);
    void removeJoint();
    void updateTheta();
    void updateTotal();
    void updateJacobian();
    void updateInvJacobian();
    void setFinalPosition(const vector<double>& p);
    const vector<double>& getPosition();
    void setBaseT(const Matrix &mat);
    void setEndT(const Matrix &mat);
    void updateTransforms(bool all=false);
    void updateTransform(int i);
    void correctOrientation();
    double& operator [](const int &i){return (*JointVector)[i].getTheta();};
    const double& initial(int i){return initialTheta[i];};
private:
    vector<Joint> * JointVector;
    vector<double> initialTheta;
    Matrix baseT;
    Matrix endT;
    Matrix Total;
    Matrix Jacobian;
    Matrix InvJacobian;
    vector<double> position;
    vector<double> finalPosition;     
};

class Legs
{
public:
	Legs();
	~Legs();
	void useLeftLeg();
	void useRightLeg();
	void liftLeg();
	void adjustYaw(double angle);
	bool setLeg(double x, double y, double z=3.0, bool flat=true);
	bool moveLeg(double dx, double dy, double dz=0.0, bool flat=true);
	void reset();
	vector<vector<float> > kick();
	void inputLeft(vector<float> input);
	void inputRight(vector<float> input);
	vector<float> outputLeft();
	vector<float> outputRight();
	vector<double> getLeftPosition(){return lLegPos;};
	vector<double> getRightPosition(){return rLegPos;};
	double getYaw(){return yaw;};
	legChoice getLegInUse(){return legInUse;};
	
		
private:
	JointSystem * LeftLeg;
	JointSystem * RightLeg;	
	vector<double> lLegPos;
	vector<double> rLegPos;
	double yaw;
	legChoice legInUse; 
};

Matrix RotX(double theta);
Matrix RotY(double theta);
Matrix RotZ(double theta);
Matrix Trans(double x, double y, double z);


#endif
