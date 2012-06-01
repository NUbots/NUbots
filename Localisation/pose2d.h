#ifndef POSE2D_H
#define POSE2D_H

class Pose2D{
		
	public:
		/// x position
		double X;
		
		/// position
		double Y;
		
		/// angular orientation
		double Theta;
	
		/// Copy Construct
		Pose2D(const double& x1, const double& y1,const double& theta1):X(x1),Y(y1),Theta(theta1)
		{}
		/// Default Constructor
		Pose2D()
		{
			X = 0.00;
			Y = 0.00;
			Theta = 0.00;
		}
		
		/// Destructor
		~Pose2D()
		{}

};

#endif
