#include "IK.h"
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include "debug.h"

Joint::Joint()
{
    alpha = 0;
    a = 0;
    theta = 0;
    d = 0;           
}

Joint::Joint(const Joint& j)
{
	alpha = j.alpha;
	a = j.a;
	theta = j.theta;
	d = j.d;
        trans = j.trans;
        diff = j.diff;
}

Joint::Joint(double alpha, double a, double theta, double d)
{
    this->alpha = alpha;
    this->a = a;
    this->theta = theta;
    this->d = d;
    trans = createTransformMatrix();
    diff = createDiffTransformMatrix();
}

Joint::~Joint()
{
}

Matrix * Joint::getTransformMatrix()
{
        return &trans;
}

Matrix * Joint::getDiffTransformMatrix()
{
        return &diff;
}

Matrix Joint::createTransformMatrix()
{ 
    Matrix transform(4,4);
    transform(0,0) = cos(theta);
    transform(0,1) = -sin(theta);
    transform(0,2) = 0;
    transform(0,3) = a;
    transform(1,0) = sin(theta)*cos(alpha);
    transform(1,1) = cos(theta)*cos(alpha);
    transform(1,2) = -sin(alpha);
    transform(1,3) = -d*sin(alpha);
    transform(2,0) = sin(theta)*sin(alpha);
    transform(2,1) = cos(theta)*sin(alpha);
    transform(2,2) = cos(alpha);
    transform(2,3) = d*cos(alpha);
    transform(3,0) = 0;
    transform(3,1) = 0;
    transform(3,2) = 0;
    transform(3,3) = 1;
    return transform;
   
}

Matrix Joint::createDiffTransformMatrix()
{   
    Matrix transform(4,4);
    transform(0,0) = -sin(theta);
    transform(0,1) = -cos(theta);
    transform(0,2) = 0;
    transform(0,3) = 0;
    transform(1,0) = cos(theta)*cos(alpha);
    transform(1,1) = -sin(theta)*cos(alpha);
    transform(1,2) = 0;
    transform(1,3) = 0;
    transform(2,0) = cos(theta)*sin(alpha);
    transform(2,1) = -sin(theta)*sin(alpha);
    transform(2,2) = 0;
    transform(2,3) = 0;
    transform(3,0) = 0;
    transform(3,1) = 0;
    transform(3,2) = 0;
    transform(3,3) = 0;
    return transform;        
} 

void Joint::updateTransforms()
{
    trans = createTransformMatrix();
    diff = createDiffTransformMatrix();
} 

Joint& Joint::operator=(const Joint& j)
{
	alpha = j.alpha;
	a = j.a;
	theta = j.theta;
	d = j.d;
        trans = j.trans;
        diff = j.diff;
	return (*this);
} 

JointSystem::JointSystem()
{
    JointVector = new vector<Joint>();
    JointVector->reserve(6);
    initialTheta.reserve(6);
    position.resize(3);
    finalPosition.resize(3);
                       
}

JointSystem::~JointSystem()
{
    delete JointVector;                         
}

void JointSystem::addJoint(Joint &j)
{
        JointVector->push_back(j);
	initialTheta.push_back((j.getTheta()));
}

void JointSystem::removeJoint()
{
    JointVector->pop_back();
    initialTheta.pop_back();
}

void JointSystem::updateTheta()
{
    
    Matrix deltaX(3,1);
    for(int i = 0; i<3; i++)
        deltaX(i,0) = (finalPosition[i]-position[i]);
    Matrix mat(InvJacobian * deltaX);
    for(int i=1; i<4; i++)
        (*this)[i] += mat(i-1,0);   
    
    //check constraints
    if((*this)[3]<0) //ensure knee joint extends in allowed direction
    {
    	double correction = 1.5*mat(2,0);
    	(*this)[3] -= correction;
    	(*this)[2] += correction;
	}
    
	updateTransforms();
}

void JointSystem::updateTotal()
{
    Matrix mat(*(*JointVector)[0].getTransformMatrix());

    for(int i = 1; i<6; i++)
    {   
        mat = mat*(*(*JointVector)[i].getTransformMatrix());
                        
    }
    Total = baseT * mat * endT;
    for(int i=0; i<3; i++)
        position[i] = Total[i][3];
}

void JointSystem::updateJacobian()
{
    Jacobian = Matrix(3, 3);
    for(size_t i=1; i<4; i++)
    {
        Matrix mat(*((*JointVector)[0].getTransformMatrix()));
        for(size_t j=1; j<6; j++)
        {   
            if(i==j)
                mat = Matrix((mat)*(*((*JointVector)[j].getDiffTransformMatrix()))); 
            else
                mat = Matrix((mat)*(*((*JointVector)[j].getTransformMatrix())));                        
        }
        mat = baseT * mat * endT;
        for(int k=0; k<3; k++)
            Jacobian[k][i-1] = mat(k,3);
    }   
}

void JointSystem::updateInvJacobian()
{
    Matrix deltaX(3,1);
    for(int i = 0; i<3; i++)
        deltaX[i][0] = (finalPosition[i]-position[i]);
    Matrix operand(Jacobian * Jacobian.transp() * deltaX);
    double damp = (dot((deltaX),(operand))/dot((operand),(operand)));
    InvJacobian = damp * Jacobian.transp();
}

void JointSystem::setFinalPosition(const vector<double>& p)
{
    for(int i=0; i<3; i++)
        finalPosition[i] = p[i];                 
}

const vector<double>& JointSystem::getPosition()
{
     return position;
}

void JointSystem::setBaseT(const Matrix &mat)
{
    baseT = mat;
}

void JointSystem::setEndT(const Matrix &mat)
{
    endT = mat;
}

void JointSystem::updateTransforms(bool all)
{
	int x, y;
	if(all)
		x=0, y=6;
	else;
		x=1, y=4;	
	for(int i = x; i<y; i++)
		(*JointVector)[i].updateTransforms();
}

void JointSystem::updateTransform(int i)
{
	(*JointVector)[i].updateTransforms();
}

void JointSystem::correctOrientation()
{	
	
        if(Total[0][2]>0)
	{
                while(Total[0][2]>0)
		{
			(*this)[4] -= 0.01;
			(*JointVector)[4].updateTransforms();
			updateTotal();
		}
                while(Total[0][2]<0)
		{
			(*this)[4] += 0.001;
			(*JointVector)[4].updateTransforms();
			updateTotal();
		}
			
	}
        if(Total[0][2]<0)
	{
                while(Total[0][2]<0)
		{
			(*this)[4] += 0.01;
			(*JointVector)[4].updateTransforms();
			updateTotal();
		}
                while(Total[0][2]>0)
		{
			(*this)[4] -= 0.001;
			(*JointVector)[4].updateTransforms();
			updateTotal();
		}
	}
	
        if(Total[1][2]>0)
	{
                while(Total[1][2]>0)
		{
			(*this)[5] += 0.01;
			(*JointVector)[5].updateTransforms();
			updateTotal();
		}
                while(Total[1][2]<0)
		{
			(*this)[5] -= 0.001;
			(*JointVector)[5].updateTransforms();
			updateTotal();
		}
			
	}
        if(Total[1][2]<0)
	{
                while(Total[1][2]<0)
		{
			(*this)[5] -= 0.01;
			(*JointVector)[5].updateTransforms();
			updateTotal();
		}
                while(Total[1][2]>0)
		{
			(*this)[5] += 0.001;
			(*JointVector)[5].updateTransforms();
			updateTotal();
		}
	}
	
}

Legs::Legs()
{
    LeftLeg = new JointSystem();
    RightLeg = new JointSystem();

    for(int i=0; i<6; i++)
    { 
        Joint lj(alphArrL[i], aArrL[i], thArrL[i], dArrL[i]);
        Joint rj(alphArrR[i], aArrR[i], thArrR[i], dArrR[i]);  
        LeftLeg->addJoint(lj);
        RightLeg->addJoint(rj);
    }

    LeftLeg->setBaseT(Trans(0, HY, -HZ));
    LeftLeg->setEndT(RotZ(PI)*RotY(-PI/2));

    RightLeg->setBaseT(Trans(0, -HY, -HZ));
    RightLeg->setEndT(RotZ(PI)*RotY(-PI/2));
    
    LeftLeg->updateTotal();
    RightLeg->updateTotal();

    lLegPos = LeftLeg->getPosition();
    rLegPos = RightLeg->getPosition();

    yaw = 0;
    
    legInUse = NONE;		
}

Legs::~Legs()
{
	delete LeftLeg;
	delete RightLeg;
}	

void Legs::useLeftLeg()
{
	legInUse = LEFT;
	
	(*RightLeg)[1] = 0.40+RightLeg->initial(1);
	RightLeg->updateTransforms();
	RightLeg->updateTotal();
	rLegPos = RightLeg->getPosition();
	RightLeg->correctOrientation();
	
	(*LeftLeg)[1] = 0.40+LeftLeg->initial(1);
	LeftLeg->updateTransforms();
	LeftLeg->updateTotal();
	lLegPos = LeftLeg->getPosition();
	LeftLeg->correctOrientation();
    
	
}

void Legs::useRightLeg()
{
	legInUse = RIGHT;
	
	(*LeftLeg)[1] = -0.40+LeftLeg->initial(1);
	LeftLeg->updateTransforms();
	LeftLeg->updateTotal();
	lLegPos = LeftLeg->getPosition();
	LeftLeg->correctOrientation();
	
	(*RightLeg)[1] = -0.40+RightLeg->initial(1);
	RightLeg->updateTransforms();
	RightLeg->updateTotal();
	rLegPos = RightLeg->getPosition();
	RightLeg->correctOrientation();
	
    
}

void Legs::liftLeg()
{
	if(legInUse==LEFT)
	{
		moveLeg(0, 3, 3);
	}
	else
	{
		if(legInUse==RIGHT)
		{
			moveLeg(0, -3, 3);
		}
	}
		
}

void Legs::adjustYaw(double angle)
{
	//increase turns in, decrease turns out
	switch(legInUse)
	{
		case NONE:
		{
			return;
		}
		case LEFT:
		{
			angle *= -0.5;
			(*RightLeg)[2] -= angle;
			break;
		}
		case RIGHT:
		{
			angle *= 0.5;
			(*LeftLeg)[2] -= angle;
			break;
		}
	}
	(*LeftLeg)[0] = (angle+LeftLeg->initial(0));
	(*RightLeg)[0] = (angle+RightLeg->initial(0));
	LeftLeg->updateTransform(0);
	LeftLeg->updateTransform(2);
	LeftLeg->updateTotal();
	RightLeg->updateTransform(0);
	RightLeg->updateTransform(2);
	RightLeg->updateTotal();
	
	lLegPos = LeftLeg->getPosition();
	rLegPos = RightLeg->getPosition();
	
	LeftLeg->correctOrientation();
	RightLeg->correctOrientation();
}

bool Legs::setLeg(double x, double y, double z, bool flat)
{
	return moveLeg(x-lLegPos[0], y-lLegPos[1], z-lLegPos[2]+rLegPos[2], flat);
}

bool Legs::moveLeg(double dx, double dy, double dz, bool flat)
{
	JointSystem * kickLeg;
	JointSystem * supportLeg;
	vector<double> * pos;
	switch(legInUse)
	{
		case NONE:
		{
			return false;
		}
		case LEFT:
		{
			kickLeg = LeftLeg;
			supportLeg = RightLeg;
			pos = &lLegPos;
			break;
		}
		case RIGHT:
		{
			kickLeg = RightLeg;
			supportLeg = LeftLeg;
			pos = &rLegPos;
			break;
		}
		default:
		{
			reset();
			return false;	
		}		
	}
	vector<double> aux(6);
	for(int i = 0; i<6; i++)
		aux[i] = (*kickLeg)[i];
		
    vector<double> current(3), final(3);
    current = final = kickLeg->getPosition();
     
    final[0]+=dx;
   	final[1]+=dy;
    final[2]+=dz;
    
    kickLeg->setFinalPosition(final);
    
    int i=0;
    double err = sqrt(pow((final[0]-current[0]),2)+pow((final[1]-current[1]),2)+pow((final[2]-current[2]),2));
    while(i<2000)
    {
        kickLeg->updateJacobian();
        kickLeg->updateInvJacobian();
        kickLeg->updateTheta();
        kickLeg->updateTotal();
        current = kickLeg->getPosition();
        err = sqrt(pow((final[0]-current[0]),2)+pow((final[1]-current[1]),2)+pow((final[2]-current[2]),2));
        i++;
        
        if(i==2000/*err<0.5*/)
        {
        	if(flat)
        		kickLeg->correctOrientation();
        	(*pos) = kickLeg->getPosition();
			return true;	
		}
    }
    for(int i = 0; i<6; i++)
		(*kickLeg)[i] = aux[i];
	return false;	
}

void Legs::reset()
{
	for(int i = 0; i<6; i++)
	{
		(*LeftLeg)[i] = thArrL[i];
		(*RightLeg)[i] = thArrR[i];
	}	
	yaw=0;
	legInUse=NONE;
	
	LeftLeg->updateTotal();
    RightLeg->updateTotal();
    
    lLegPos = LeftLeg->getPosition();
    rLegPos = RightLeg->getPosition();	
}

vector<vector<float> > Legs::kick()
{
	vector<vector<float> > poseList;
	poseList.resize(4);
	for(int i = 0; i<4; i++)
		poseList[i].resize(6);
	setLeg(-5, 10.6, 6.8);
	poseList[0] = outputLeft();
	poseList[1] = outputRight();
	setLeg(13, 10.6, 8.7);
	poseList[2] = outputLeft();
	poseList[3] = outputRight();

	return poseList;
	
}

void Legs::inputLeft(vector<float> input)
{

	(*LeftLeg)[1] = input[0]+LeftLeg->initial(1);
	(*LeftLeg)[2] = input[1];
	(*LeftLeg)[0] = input[2]+LeftLeg->initial(0);
	(*LeftLeg)[3] = input[3];
	(*LeftLeg)[5] = input[4];
	(*LeftLeg)[4] = input[5];
	LeftLeg->updateTransforms(true);
}

void Legs::inputRight(vector<float> input)
{

	(*RightLeg)[1] = input[0]+RightLeg->initial(1);
	(*RightLeg)[2] = input[1];
	(*RightLeg)[0] = input[2]+RightLeg->initial(0);
	(*RightLeg)[3] = input[3];
	(*RightLeg)[5] = input[4];
	(*RightLeg)[4] = input[5];
	RightLeg->updateTransforms(true);

}

vector<float> Legs::outputLeft()
{
	vector<float> output(6);
	output[0] = (*LeftLeg)[1]-LeftLeg->initial(1);
	output[1] = (*LeftLeg)[2];
	output[2] = (*LeftLeg)[0]-LeftLeg->initial(0);
	output[3] = (*LeftLeg)[3];
	output[4] = (*LeftLeg)[5];
	output[5] = (*LeftLeg)[4];
	return output;
}

vector<float> Legs::outputRight()
{
	vector<float> output(6);
	output[0] = (*RightLeg)[1]-RightLeg->initial(1);
	output[1] = (*RightLeg)[2];
	output[2] = (*RightLeg)[0]-RightLeg->initial(0);
	output[3] = (*RightLeg)[3];
	output[4] = (*RightLeg)[5];
	output[5] = (*RightLeg)[4];
	return output;
}	

Matrix RotX(double theta)
{
    Matrix mat(4,4);
    mat(0,0) = 1;
    mat(0,1) = 0;
    mat(0,2) = 0;
    mat(0,3) = 0;
    mat(1,0) = 0;
    mat(1,1) = cos(theta);
    mat(1,2) = -sin(theta);
    mat(1,3) = 0;
    mat(2,0) = 0;
    mat(2,1) = sin(theta);
    mat(2,2) = cos(theta);
    mat(2,3) = 0;
    mat(3,0) = 0;
    mat(3,1) = 0;
    mat(3,2) = 0;
    mat(3,3) = 1;
    return mat;        
}

Matrix RotY(double theta)
{
    Matrix mat(4,4);
    mat(0,0) = cos(theta);
    mat(0,1) = 0;
    mat(0,2) = sin(theta);
    mat(0,3) = 0;
    mat(1,0) = 0;
    mat(1,1) = 1;
    mat(1,2) = 0;
    mat(1,3) = 0;
    mat(2,0) = -sin(theta);
    mat(2,1) = 0;
    mat(2,2) = cos(theta);
    mat(2,3) = 0;
    mat(3,0) = 0;
    mat(3,1) = 0;
    mat(3,2) = 0;
    mat(3,3) = 1;
    return mat;    
}

Matrix RotZ(double theta)
{
    Matrix mat(4,4);
    mat(0,0) = cos(theta);
    mat(0,1) = -sin(theta);
    mat(0,2) = 0;
    mat(0,3) = 0;
    mat(1,0) = sin(theta);
    mat(1,1) = cos(theta);
    mat(1,2) = 0;
    mat(1,3) = 0;
    mat(2,0) = 0;
    mat(2,1) = 0;
    mat(2,2) = 1;
    mat(2,3) = 0;
    mat(3,0) = 0;
    mat(3,1) = 0;
    mat(3,2) = 0;
    mat(3,3) = 1;
    return mat;    
}

Matrix Trans(double x, double y, double z)
{
    Matrix mat(4,4);
    mat(0,0) = 1;
    mat(0,1) = 0;
    mat(0,2) = 0;
    mat(0,3) = x;
    mat(1,0) = 0;
    mat(1,1) = 1;
    mat(1,2) = 0;
    mat(1,3) = y;
    mat(2,0) = 0;
    mat(2,1) = 0;
    mat(2,2) = 1;
    mat(2,3) = z;
    mat(3,0) = 0;
    mat(3,1) = 0;
    mat(3,2) = 0;
    mat(3,3) = 1;
    return mat;    
} 
