#include "balldetection.h"

void BallDetection::detectBall()
{
    //cout << "IT'S WORKING" << endl;
    NUImage img = VisionBlackboard::getInstance()->getOriginalImage();
    const LookUpTable& lut = VisionBlackboard::getInstance()->getLUT();
    Mat cvimg;
    lut.classifyImage(img, cvimg);

    // BEGIN BALL DETECTION -----------------------------------------------------------------

    VisionBlackboard* vbb = VisionBlackboard::getInstance();
//    const vector<Transition>& transitions_const = vbb->getVerticalTransitions(VisionFieldObject::BALL);
    vector<Transition> transitions = vbb->getVerticalTransitions(VisionFieldObject::BALL);
    //vector<Transitions>

    const vector<PointType>& horizon = vbb->getHorizonPoints();

    // Arithmetic mean and std. dev. throw-out

    vector<Transition>::iterator it;
    it = transitions.begin();
    while (it < transitions.end()) {
        bool flag = 0;
        for (int i = 0; i < horizon.size(); i++) {
            if (horizon.at(i).y >= it->getLocation().y) {
                it = transitions.erase(it);
                flag = 1;
                break;
            }
        }
        if (!flag)
            it++;
    }


    // Geometric mean
    double x_pos = 1,
           y_pos = 1;
    for (int i = 0; i < transitions.size(); i++) {
        int x = transitions.at(i).getLocation().x,
            y = transitions.at(i).getLocation().y;

        x_pos *= x;
        y_pos *= y;
    }
    x_pos = pow(x_pos, 1.0/transitions.size());
    y_pos = pow(y_pos, 1.0/transitions.size());


    // END BALL DETECTION -------------------------------------------------------------------

    circle(cvimg, PointType((int)x_pos,(int)y_pos), 5, Scalar(0,0,255), 2);

    namedWindow("BALL DETECTION", CV_WINDOW_KEEPRATIO);
    imshow("BALL DETECTION", cvimg);
}
