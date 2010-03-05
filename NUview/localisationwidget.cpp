#include "localisationwidget.h"
#include <QDebug>


LocalisationWidget::LocalisationWidget(QWidget *parent): QDockWidget(parent)
{
    for(int i=0;i<6;i++)
        paramaters[i] = 0;
    for(int j=0;j<22;j++)
        joints[j] = 0;

    setObjectName(tr("Localisation"));
    setWindowTitle(tr("Localisation"));
    createWidgets();
    createLayout();
    createConnections();

    createLines();
    createGoals();
    ball = Sphere(0,0);
    bottomCamera = false;
    leftLegOnGround = true;
    rightLegOnGround = true;

    anglePoint = WMPoint();
}
LocalisationWidget::~LocalisationWidget()
{
    delete xSpinBox;
    delete ySpinBox;
    delete thetaSpinBox;
    delete headYawSpinBox;
    delete headPitchSpinBox;
    delete lHipPitchSpinBox;
    delete lHipRollSpinBox;
    delete rHipPitchSpinBox;
    delete rHipRollSpinBox;
}
void LocalisationWidget::createWidgets()
{
    xSpinBox = new QDoubleSpinBox;
    ySpinBox = new QDoubleSpinBox;
    thetaSpinBox = new QDoubleSpinBox;
    ballXSpinBox = new QDoubleSpinBox;
    ballYSpinBox = new QDoubleSpinBox;

    headYawSpinBox = new QDoubleSpinBox;
    headPitchSpinBox = new QDoubleSpinBox;
    lHipPitchSpinBox = new QDoubleSpinBox;
    lHipRollSpinBox = new QDoubleSpinBox;
    rHipPitchSpinBox = new QDoubleSpinBox;
    rHipRollSpinBox = new QDoubleSpinBox;

    anglePointXSpinBox = new QDoubleSpinBox;
    anglePointYSpinBox = new QDoubleSpinBox;
    anglePointZSpinBox = new QDoubleSpinBox;

    calculateAngle = new QPushButton("Calculate Angle");

    xLabel = new QLabel("Robot's x");
    yLabel = new QLabel("Robot's y");
    thetaLabel = new QLabel("Robot's Bearing");
    ballXLabel = new QLabel("Ball's x");
    ballYLabel = new QLabel("Ball's y");

    headYawLabel = new QLabel("Head Yaw");
    headPitchLabel = new QLabel("Head Pitch");
    lHipPitchLabel = new QLabel("Left Hip Pitch");
    lHipRollLabel = new QLabel("Left Hip Roll");
    rHipPitchLabel = new QLabel("Right Hip Pitch");
    rHipRollLabel = new QLabel("Right Hip Roll");

    anglePointXLabel = new QLabel("Point x:");
    anglePointYLabel = new QLabel("Point y:");
    anglePointZLabel = new QLabel("Point z:");

    totalAngleLabel = new QLabel("Total angle:");
    yawAngleLabel = new QLabel("Yaw angle:");
    pitchAngleLabel = new QLabel("Pitch angle:");

    totalAngle = new QLabel("-");
    yawAngle = new QLabel("-");
    pitchAngle = new QLabel("-");
    
    headYawJointLabel = new QLabel("Head Yaw:");
    headPitchJointLabel = new QLabel("Head Pitch:");

    lHipYawPitchJointLabel = new QLabel("L Hip YawPitch:");
    lHipRollJointLabel = new QLabel("L Hip Roll:");
    lHipPitchJointLabel = new QLabel("L Hip Pitch:");
    lKneePitchJointLabel = new QLabel("L Knee Pitch:");
    lAnklePitchJointLabel = new QLabel("L Ankle Pitch:");
    lAnkleRollJointLabel = new QLabel("L Ankle Roll:");

    rHipYawPitchJointLabel = new QLabel("R Hip YawPitch:");
    rHipRollJointLabel = new QLabel("R Hip Roll:");
    rHipPitchJointLabel = new QLabel("R Hip Pitch:");
    rKneePitchJointLabel = new QLabel("R Knee Pitch:");
    rAnklePitchJointLabel = new QLabel("R Ankle Pitch:");
    rAnkleRollJointLabel = new QLabel("R Ankle Roll:");

    cameraLabel = new QLabel("Camera:");

    headYawText = new QLabel("0.00");
    headPitchText = new QLabel("0.00");
    lHipPitchYawText = new QLabel("0.00");
    lHipRollText = new QLabel("0.00");
    lHipPitchText = new QLabel("0.00");
    lKneePitchText = new QLabel("0.00");
    lAnklePitchText = new QLabel("0.00");;
    lAnkleRollText = new QLabel("0.00");
    rHipPitchYawText = new QLabel("0.00");
    rHipRollText = new QLabel("0.00");
    rHipPitchText = new QLabel("0.00");
    rKneePitchText = new QLabel("0.00");
    rAnklePitchText = new QLabel("0.00");;
    rAnkleRollText = new QLabel("0.00");

    cameraText = new QLabel("Top");

    commonPointsComboBox = new QComboBox;
    commonPointsComboBox->addItem(tr("Custom"),0);
    commonPointsComboBox->addItem(tr("Yellow Goal"),1);
    commonPointsComboBox->addItem(tr("Blue Goal"),2);
    commonPointsComboBox->addItem(tr("Ball"),3);

    xSpinBox->setRange(-302.5,302.5);
    ySpinBox->setRange(-202.5,202.5);
    thetaSpinBox->setRange(-360,360);
    ballXSpinBox->setRange(-302.5,302.5);
    ballYSpinBox->setRange(-202.5,202.5);

    headYawSpinBox->setRange(-20,20);
    headPitchSpinBox->setRange(-20,20);
    lHipPitchSpinBox->setRange(-20,20);
    lHipRollSpinBox->setRange(-20,20);
    rHipPitchSpinBox->setRange(-20,20);
    rHipRollSpinBox->setRange(-20,20);

    anglePointXSpinBox->setRange(-302.5,302.5);
    anglePointYSpinBox->setRange(-202.5,202.5);
    anglePointZSpinBox->setRange(0,80.0);

    headYawSpinBox->setSingleStep(0.1);
    headPitchSpinBox->setSingleStep(0.1);
    lHipPitchSpinBox->setSingleStep(0.1);
    lHipRollSpinBox->setSingleStep(0.1);
    rHipPitchSpinBox->setSingleStep(0.1);
    rHipRollSpinBox->setSingleStep(0.1);

    anglePointZSpinBox->setMinimumWidth(59);

}
void LocalisationWidget::createLayout()
{
    localisationLayout = new QGridLayout;
    for(int count = 0;count < 5;count++)
    {
        localisationLayout->setColumnMinimumWidth(count,70);
    }
    localisationGroupBox = new QGroupBox(tr("Localisation"));
    localisationLayout->addWidget(xLabel,0,0,Qt::AlignCenter);
    localisationLayout->addWidget(yLabel,0,1,Qt::AlignCenter);
    localisationLayout->addWidget(thetaLabel,0,2,Qt::AlignCenter);
    localisationLayout->addWidget(ballXLabel,0,3,Qt::AlignCenter);
    localisationLayout->addWidget(ballYLabel,0,4,Qt::AlignCenter);

    localisationLayout->addWidget(xSpinBox,1,0,Qt::AlignCenter);
    localisationLayout->addWidget(ySpinBox,1,1,Qt::AlignCenter);
    localisationLayout->addWidget(thetaSpinBox,1,2,Qt::AlignCenter);
    localisationLayout->addWidget(ballXSpinBox,1,3,Qt::AlignCenter);
    localisationLayout->addWidget(ballYSpinBox,1,4,Qt::AlignCenter);

    localisationGroupBox->setLayout(localisationLayout);

    paramaterLayout = new QGridLayout;
    for(int count = 0;count<6;count++)
    {
        paramaterLayout->setColumnMinimumWidth(count,65);
    }
    paramaterGroupBox = new QGroupBox(tr("Calibration Paramaters"));

    paramaterLayout->addWidget(headYawLabel,0,0,Qt::AlignCenter);
    paramaterLayout->addWidget(headPitchLabel,0,1,Qt::AlignCenter);
    paramaterLayout->addWidget(lHipPitchLabel,0,2,Qt::AlignCenter);
    paramaterLayout->addWidget(lHipRollLabel,0,3,Qt::AlignCenter);
    paramaterLayout->addWidget(rHipPitchLabel,0,4,Qt::AlignCenter);
    paramaterLayout->addWidget(rHipRollLabel,0,5,Qt::AlignCenter);

    paramaterLayout->addWidget(headYawSpinBox,1,0,Qt::AlignCenter);
    paramaterLayout->addWidget(headPitchSpinBox,1,1,Qt::AlignCenter);
    paramaterLayout->addWidget(lHipPitchSpinBox,1,2,Qt::AlignCenter);
    paramaterLayout->addWidget(lHipRollSpinBox,1,3,Qt::AlignCenter);
    paramaterLayout->addWidget(rHipPitchSpinBox,1,4,Qt::AlignCenter);
    paramaterLayout->addWidget(rHipRollSpinBox,1,5,Qt::AlignCenter);

    paramaterGroupBox->setLayout(paramaterLayout);

    angleToPointLayout = new QGridLayout;
    angleToPointGroupBox = new QGroupBox(tr("Angle to Point"));

    angleToPointLayout->setColumnMinimumWidth(3,35);

    angleToPointLayout->addWidget(anglePointXLabel,1,0,Qt::AlignRight);
    angleToPointLayout->addWidget(anglePointYLabel,2,0,Qt::AlignRight);
    angleToPointLayout->addWidget(anglePointZLabel,3,0,Qt::AlignRight);

    angleToPointLayout->addWidget(commonPointsComboBox,0,1);
    angleToPointLayout->addWidget(anglePointXSpinBox,1,1,Qt::AlignLeft);
    angleToPointLayout->addWidget(anglePointYSpinBox,2,1,Qt::AlignLeft);
    angleToPointLayout->addWidget(anglePointZSpinBox,3,1,Qt::AlignLeft);

    angleToPointLayout->addWidget(calculateAngle,0,2);
    angleToPointLayout->addWidget(totalAngleLabel,1,2,Qt::AlignRight);
    angleToPointLayout->addWidget(yawAngleLabel,2,2,Qt::AlignRight);
    angleToPointLayout->addWidget(pitchAngleLabel,3,2,Qt::AlignRight);

    angleToPointLayout->addWidget(totalAngle,1,3,Qt::AlignRight);
    angleToPointLayout->addWidget(yawAngle,2,3,Qt::AlignRight);
    angleToPointLayout->addWidget(pitchAngle,3,3,Qt::AlignRight);

    angleToPointGroupBox->setLayout(angleToPointLayout);


    groupLayout = new QGridLayout;
    groupLayout->addWidget(paramaterGroupBox,0,0,Qt::AlignLeft);
    groupLayout->addWidget(localisationGroupBox,1,0,Qt::AlignLeft);

    jointsLayout = new QGridLayout;
    jointsGroupBox = new QGroupBox(tr("Joint Angles"));

    jointsLayout->setColumnMinimumWidth(1,35);
    jointsLayout->setColumnMinimumWidth(3,35);
    jointsLayout->setColumnMinimumWidth(5,35);

    jointsLayout->addWidget(headYawJointLabel,0,0,Qt::AlignRight);
    jointsLayout->addWidget(headPitchJointLabel,1,0,Qt::AlignRight);
    jointsLayout->addWidget(cameraLabel,2,0,Qt::AlignRight);
    jointsLayout->addWidget(lHipYawPitchJointLabel,3,0,Qt::AlignRight);
    jointsLayout->addWidget(rHipYawPitchJointLabel,4,0,Qt::AlignRight);

    jointsLayout->addWidget(lHipRollJointLabel,0,2,Qt::AlignRight);
    jointsLayout->addWidget(lHipPitchJointLabel,1,2,Qt::AlignRight);
    jointsLayout->addWidget(lKneePitchJointLabel,2,2,Qt::AlignRight);
    jointsLayout->addWidget(lAnklePitchJointLabel,3,2,Qt::AlignRight);
    jointsLayout->addWidget(lAnkleRollJointLabel,4,2,Qt::AlignRight);

    jointsLayout->addWidget(rHipRollJointLabel,0,4,Qt::AlignRight);
    jointsLayout->addWidget(rHipPitchJointLabel,1,4,Qt::AlignRight);
    jointsLayout->addWidget(rKneePitchJointLabel,2,4,Qt::AlignRight);
    jointsLayout->addWidget(rAnklePitchJointLabel,3,4,Qt::AlignRight);
    jointsLayout->addWidget(rAnkleRollJointLabel,4,4,Qt::AlignRight);

    jointsLayout->addWidget(headYawText,0,1,Qt::AlignRight);
    jointsLayout->addWidget(headPitchText,1,1,Qt::AlignRight);
    jointsLayout->addWidget(cameraText,2,1,Qt::AlignCenter);
    jointsLayout->addWidget(lHipPitchYawText,3,1,Qt::AlignRight);
    jointsLayout->addWidget(rHipPitchYawText,4,1,Qt::AlignRight);

    jointsLayout->addWidget(lHipRollText,0,3,Qt::AlignRight);
    jointsLayout->addWidget(lHipPitchText,1,3,Qt::AlignRight);
    jointsLayout->addWidget(lKneePitchText,2,3,Qt::AlignRight);
    jointsLayout->addWidget(lAnklePitchText,3,3,Qt::AlignRight);
    jointsLayout->addWidget(lAnkleRollText,4,3,Qt::AlignRight);

    jointsLayout->addWidget(rHipRollText,0,5,Qt::AlignRight);
    jointsLayout->addWidget(rHipPitchText,1,5,Qt::AlignRight);
    jointsLayout->addWidget(rKneePitchText,2,5,Qt::AlignRight);
    jointsLayout->addWidget(rAnklePitchText,3,5,Qt::AlignRight);
    jointsLayout->addWidget(rAnkleRollText,4,5,Qt::AlignRight);

    jointsGroupBox->setLayout(jointsLayout);

    totalLayout = new QGridLayout;
    totalLayout->addLayout(groupLayout,0,0,Qt::AlignLeft);
    totalLayout->addWidget(angleToPointGroupBox,0,1,Qt::AlignLeft);
    totalLayout->addWidget(jointsGroupBox,0,2,Qt::AlignLeft);


    window = new QWidget;

    window->setLayout(totalLayout);
    setWidget(window);
}
void LocalisationWidget::createConnections()
{
    connect(xSpinBox,SIGNAL(valueChanged(double)), this, SLOT(xChanged(double)));
    connect(ySpinBox,SIGNAL(valueChanged(double)), this, SLOT(yChanged(double)));
    connect(thetaSpinBox,SIGNAL(valueChanged(double)), this, SLOT(thetaChanged(double)));
    connect(ballXSpinBox,SIGNAL(valueChanged(double)), this, SLOT(ballXChanged(double)));
    connect(ballYSpinBox,SIGNAL(valueChanged(double)), this, SLOT(ballYChanged(double)));

    connect(headYawSpinBox,SIGNAL(valueChanged(double)),this,SLOT(headYawChanged(double)));
    connect(headPitchSpinBox,SIGNAL(valueChanged(double)),this,SLOT(headPitchChanged(double)));
    connect(lHipPitchSpinBox,SIGNAL(valueChanged(double)),this,SLOT(lHipPitchChanged(double)));
    connect(lHipRollSpinBox,SIGNAL(valueChanged(double)),this,SLOT(lHipRollChanged(double)));
    connect(rHipPitchSpinBox,SIGNAL(valueChanged(double)),this,SLOT(rHipPitchChanged(double)));
    connect(rHipRollSpinBox,SIGNAL(valueChanged(double)),this,SLOT(rHipRollChanged(double)));

    connect(anglePointXSpinBox,SIGNAL(valueChanged(double)),this,SLOT(angleXChanged(double)));
    connect(anglePointYSpinBox,SIGNAL(valueChanged(double)),this,SLOT(angleYChanged(double)));
    connect(anglePointZSpinBox,SIGNAL(valueChanged(double)),this,SLOT(angleZChanged(double)));

    connect(calculateAngle,SIGNAL(clicked()),this,SLOT(calculateAngleClicked()));
    connect(commonPointsComboBox, SIGNAL(currentIndexChanged(int)),this,SLOT(commonPointsChanged(int)));
}

void LocalisationWidget::xChanged(double newX)
{
    xInput = newX;
    calculateLines();
}
void LocalisationWidget::yChanged(double newY)
{
    yInput = newY;
    calculateLines();
}
void LocalisationWidget::thetaChanged(double newTheta)
{
    thetaInput = newTheta;
    calculateLines();
}
void LocalisationWidget::ballXChanged(double newBallX)
{
    ballX = newBallX;
    calculateLines();
    if(commonPointsComboBox->currentIndex() == 3)
        anglePointXSpinBox->setValue(newBallX);

}
void LocalisationWidget::ballYChanged(double newBallY)
{
    ballY = newBallY;
    calculateLines();
    if(commonPointsComboBox->currentIndex() == 3)
        anglePointYSpinBox->setValue(newBallY);
}

void LocalisationWidget::headYawChanged(double newHeadYaw)
{
    paramaters[0] = newHeadYaw;
    calculateLines();
}
void LocalisationWidget::headPitchChanged(double newHeadPitch)
{
    paramaters[1] = newHeadPitch;
    calculateLines();
}
void LocalisationWidget::lHipPitchChanged(double newLHipPitch)
{
    paramaters[2] = newLHipPitch;
    calculateLines();
}
void LocalisationWidget::lHipRollChanged(double newLHipRoll)
{
    paramaters[3] = newLHipRoll;
    calculateLines();
}
void LocalisationWidget::rHipPitchChanged(double newRHipPitch)
{
    paramaters[4] = newRHipPitch;
    calculateLines();
}
void LocalisationWidget::rHipRollChanged(double newRHipRoll)
{
    paramaters[5] = newRHipRoll;
    calculateLines();
}

void LocalisationWidget::calculateLines()
{
    countVisibleLines = 0;

    camera = CameraMatrix(paramaters,joints,xInput,yInput,thetaInput,bottomCamera);

    if(leftLegOnGround)
    {

        for (int i = 0;i<74;i++)
        {
            WMLine line_transform = line_array[i];

            line_transform.setStart(camera.getLeft() * line_transform.getStart().get());
            line_transform.setEnd(camera.getLeft() * line_transform.getEnd().get());
            displayLines(line_transform);
        }
        for (int j = 0;j<4;j++)
        {
            Cylinder goal_transform = goal_array[j];
            goal_transform.setMid(camera.getLeft() * goal_transform.getMid().getStart().get(),camera.getLeft() * goal_transform.getMid().getEnd().get());
            displayGoals(goal_transform);
        }
        emit updateLocalisationLine(visibleLines,countVisibleLines,GLDisplay::wmLeftLeg);
        countVisibleLines = 0;
    }
    else
    {
        emit removeLocalisationLine(GLDisplay::wmLeftLeg);
    }

    if(rightLegOnGround)
    {
        for (int i = 0;i<74;i++)
        {
            WMLine line_transform = line_array[i];

            line_transform.setStart(camera.getRight() * line_transform.getStart().get());
            line_transform.setEnd(camera.getRight() * line_transform.getEnd().get());
            displayLines(line_transform);
         }
         for (int j = 0;j<4;j++)
         {
            Cylinder goal_transform = goal_array[j];
            goal_transform.setMid(camera.getRight() * goal_transform.getMid().getStart().get(),camera.getRight() * goal_transform.getMid().getEnd().get());
            displayGoals(goal_transform);
          }
        emit updateLocalisationLine(visibleLines,countVisibleLines,GLDisplay::wmRightLeg);
    }
    else
    {
        emit removeLocalisationLine(GLDisplay::wmRightLeg);
    }

    ball = Sphere(ballX,ballY);
    if(leftLegOnGround)
    {
        ball.setCentre(camera.getLeft() * ball.getCentre().get());
        displayBall(ball);
    }
    else if(rightLegOnGround)
    {
        ball.setCentre(camera.getRight() * ball.getCentre().get());
        displayBall(ball);
    }

}

void LocalisationWidget::displayLines(WMLine newLine)
{
        if (Clip3D(newLine,30,1500))
        {
            //Normalise
            newLine.normalise();

            //Normal to screen coords
            if (Clip2D(newLine))
            {
                newLine.screenCoords();
              
                WMPoint tp1(roundDouble(newLine.getStart().getx() / 2),roundDouble(newLine.getStart().gety() / 2));
                WMPoint tp2(roundDouble(newLine.getEnd().getx() / 2),roundDouble(newLine.getEnd().gety() / 2));

                visibleLines[countVisibleLines] = WMLine(tp1,tp2);
                countVisibleLines++;
            }
         }
}

void LocalisationWidget::displayGoals(Cylinder newCylinder)
{
    newCylinder.createSides();
    WMLine left = newCylinder.getLeft();
    WMLine right = newCylinder.getRight();

    if (Clip3D(left,30,1500))
    {
        left.normalise();
        if (Clip2D(left))
        {
            left.screenCoords();

            WMPoint tp1(roundDouble(left.getStart().getx() / 2),roundDouble(left.getStart().gety() / 2));
            WMPoint tp2(roundDouble(left.getEnd().getx() / 2),roundDouble(left.getEnd().gety() / 2));

            visibleLines[countVisibleLines] = WMLine(tp1,tp2);
            countVisibleLines++;
        }
    }

    if (Clip3D(right,30,1500))
    {
        right.normalise();
        if (Clip2D(right))
        {
            right.screenCoords();

            WMPoint tp1(roundDouble(right.getStart().getx() / 2),roundDouble(right.getStart().gety() / 2));
            WMPoint tp2(roundDouble(right.getEnd().getx() / 2),roundDouble(right.getEnd().gety() / 2));

            visibleLines[countVisibleLines] = WMLine(tp1,tp2);
            countVisibleLines++;
        }
    }

}
void LocalisationWidget::displayBall(Sphere newSphere)
{
    newSphere.createSides();

    WMLine top = newSphere.getTop();
    WMLine bottom = newSphere.getBottom();
    WMLine mid = newSphere.getMid();


    if (Clip3D(newSphere,30,1500))
    {
        top.normalise();
        bottom.normalise();
        mid.normalise();

        top.screenCoords();
        bottom.screenCoords();
        mid.screenCoords();

        float cx = float(mid.getStart().getx()/2);
        float cy = float(mid.getStart().gety()/2);
        float r = float(fabs(top.getStart().gety() - mid.getStart().gety()) / 2);
        emit updateLocalisationBall(cx,cy,r, GLDisplay::wmBall);
    }
}
int LocalisationWidget::roundDouble(double newInt)
{
    // If number is positive, add 0.5 then truncate with 'int()' to produce rounded
    // whole number.  If negative number, subtract 0.5 for the same result.
    return int(newInt > 0.0 ? newInt + 0.5 : newInt - 0.5);
}
void LocalisationWidget::angleToPoint(WMPoint point)
{
    Matrix robotAxisTransform = Matrix(4,4);
    robotAxisTransform[0][2] = 1;
    robotAxisTransform[1][0] = -1;
    robotAxisTransform[2][1] = 1;
    robotAxisTransform[3][3] = 1;
    Matrix robotAxisCam = robotAxisTransform * camera.getLeft();
    point = WMPoint(robotAxisCam * point.get());
    double pNormXYZ = sqrt(pow(point.getx(),2) + pow(point.gety(),2) + pow(point.getz(),2));
    double pNormYaw = sqrt(pow(point.getx(),2) + pow(point.gety(),2));
    double pNormPitch = sqrt(pow(point.getx(),2) + pow(point.getz(),2));
    double angleXYZ = 57.2957795 * acos(point.getx() / pNormXYZ);
    double angleYaw = 57.2957795 * acos(point.getx() / pNormYaw);
    double anglePitch = 57.2957795 * acos(point.getx() / pNormPitch);

    if(point.gety()<0)
        angleYaw = -angleYaw;

    if(point.getx() < 0)
        anglePitch = 180 - anglePitch;
    if(point.getz()<0)
        anglePitch *= -1;

    totalAngle->setText(doubleToString(angleXYZ));
    yawAngle->setText(doubleToString(angleYaw));
    pitchAngle->setText(doubleToString(anglePitch));

}

void LocalisationWidget::frameChange(const double* jointSensors, bool camera,const double* touchSensors)
{
    for(int i=0;i<22;i++)
    {
        joints[i] = jointSensors[i];
    }
    bottomCamera = camera;

    double sumLeft = touchSensors[0] + touchSensors[1] + touchSensors[2] + touchSensors[3];
    double sumRight = touchSensors[6] + touchSensors[7] + touchSensors[8] + touchSensors[9];

    leftLegOnGround = (sumLeft < 9000);
    rightLegOnGround = (sumRight < 9000);
    calculateLines();
    updateJoints();
}
void LocalisationWidget::angleXChanged(double newAngleX)
{
    anglePoint.setx(newAngleX);
}
void LocalisationWidget::angleYChanged(double newAngleY)
{
    anglePoint.sety(newAngleY);
}
void LocalisationWidget::angleZChanged(double newAngleZ)
{
    anglePoint.setz(newAngleZ);
}
void LocalisationWidget::calculateAngleClicked()
{
    angleToPoint(anglePoint);
}
void LocalisationWidget::commonPointsChanged(int newIndex)
{
    switch(newIndex)
    {
    case 0:
        anglePoint = WMPoint(0.0,0.0,0.0);
        anglePointXSpinBox->setValue(0.0);
        anglePointYSpinBox->setValue(0.0);
        anglePointZSpinBox->setValue(0.0);
        break;
    case 1:
        anglePoint = WMPoint(302.5,0.0,40.0);
        anglePointXSpinBox->setValue(302.5);
        anglePointYSpinBox->setValue(0.0);
        anglePointZSpinBox->setValue(40.0);
        break;
    case 2:
        anglePoint = WMPoint(-302.5,0.0,40.0);
        anglePointXSpinBox->setValue(-302.5);
        anglePointYSpinBox->setValue(0.0);
        anglePointZSpinBox->setValue(40.0);
        break;
    case 3:
        anglePoint = WMPoint(ballX,ballY,3.25);
        anglePointXSpinBox->setValue(ballX);
        anglePointYSpinBox->setValue(ballY);
        anglePointZSpinBox->setValue(3.25);
        break;
    }
}
QString LocalisationWidget::doubleToString(double newString)
{
    QString string;
    string.sprintf("%.2f",newString);
    return string;
}
void LocalisationWidget::updateJoints()
{
    headYawText->setText(doubleToString(joints[0]));
    headPitchText->setText(doubleToString(joints[1]));

    lHipPitchYawText->setText(doubleToString(joints[10]));
    rHipPitchYawText->setText(doubleToString(joints[16]));

    lHipRollText->setText(doubleToString(joints[11]));
    lHipPitchText->setText(doubleToString(joints[12]));
    lKneePitchText->setText(doubleToString(joints[13]));
    lAnklePitchText->setText(doubleToString(joints[14]));
    lAnkleRollText->setText(doubleToString(joints[15]));

    rHipRollText->setText(doubleToString(joints[17]));
    rHipPitchText->setText(doubleToString(joints[18]));
    rKneePitchText->setText(doubleToString(joints[19]));
    rAnklePitchText->setText(doubleToString(joints[20]));
    rAnkleRollText->setText(doubleToString(joints[21]));

    if (bottomCamera)
        cameraText->setText("Bot");
    else
        cameraText->setText("Top");
}
void LocalisationWidget::createLines()
{
    //All Lines go from highest to lowest in their respective directions
    //Convention: positive x is yellow, positive y is left

    //Points

    //Yellow sideLine Points
    WMPoint cornerYellowLeftOuter = WMPoint(302.5,202.5);
    WMPoint cornerYellowLeftInner = WMPoint(297.5,197.5);
    WMPoint cornerYellowRightOuter = WMPoint(302.5,-202.5);
    WMPoint cornerYellowRightInner = WMPoint(297.5,-197.5);

    //Yellow goalLine Points
    WMPoint goalWMLineYellowLeftOuter = WMPoint(302.5,152.5);
    WMPoint goalWMLineYellowLeftInner = WMPoint(302.5,147.5);
    WMPoint goalWMLineYellowRightOuter = WMPoint(302.5,-152.5);
    WMPoint goalWMLineYellowRightInner = WMPoint(302.5,-147.5);

    //Yellow penalty box Points
    WMPoint penaltyYellowLeftOuter = WMPoint(237.5,152.5);
    WMPoint penaltyYellowLeftInner = WMPoint(242.5,147.5);
    WMPoint penaltyYellowRightOuter = WMPoint(237.5,-152.5);
    WMPoint penaltyYellowRightInner = WMPoint(242.5,-147.5);

    //MidLine Points
    WMPoint sideYellowLeft = WMPoint(2.5,197.5);
    WMPoint sideYellowRight = WMPoint(2.5,-197.5);
    WMPoint sideBlueLeft = WMPoint(-2.5,197.5);
    WMPoint sideBlueRight = WMPoint(-2.5,-197.5);

    //Blue sideLine Points
    WMPoint cornerBlueLeftOuter = WMPoint(-302.5,202.5);
    WMPoint cornerBlueLeftInner = WMPoint(-297.5,197.5);
    WMPoint cornerBlueRightOuter = WMPoint(-302.5,-202.5);
    WMPoint cornerBlueRightInner = WMPoint(-297.5,-197.5);

    //Blue goalLine Points
    WMPoint goalWMLineBlueLeftOuter = WMPoint(-302.5,152.5);
    WMPoint goalWMLineBlueLeftInner = WMPoint(-302.5,147.5);
    WMPoint goalWMLineBlueRightOuter = WMPoint(-302.5,-152.5);
    WMPoint goalWMLineBlueRightInner = WMPoint(-302.5,-147.5);

    //Blue penalty box Points
    WMPoint penaltyBlueLeftOuter = WMPoint(-237.5,152.5);
    WMPoint penaltyBlueLeftInner = WMPoint(-242.5,147.5);
    WMPoint penaltyBlueRightOuter = WMPoint(-237.5,-152.5);
    WMPoint penaltyBlueRightInner = WMPoint(-242.5,-147.5);

    //Yellow Cross Points
    WMPoint yellowCrossHorizontalOutsideLeft = WMPoint(125,2.5);
    WMPoint yellowCrossHorizontalOutsideRight = WMPoint(125,-2.5);
    WMPoint yellowCrossHorizontalInsideLeft = WMPoint(115,2.5);
    WMPoint yellowCrossHorizontalInsideRight = WMPoint(115,-2.5);
    WMPoint yellowCrossVerticalLeftOutside = WMPoint(122.5,5);
    WMPoint yellowCrossVerticalLeftInside = WMPoint(117.5,5);
    WMPoint yellowCrossVerticalRightOutside = WMPoint(122.5,-5);
    WMPoint yellowCrossVerticalRightInside = WMPoint(117.5,-5);

    //Blue Cross Points
    WMPoint blueCrossHorizontalOutsideLeft = WMPoint(-125,2.5);
    WMPoint blueCrossHorizontalOutsideRight = WMPoint(-125,-2.5);
    WMPoint blueCrossHorizontalInsideLeft = WMPoint(-115,2.5);
    WMPoint blueCrossHorizontalInsideRight = WMPoint(-115,-2.5);
    WMPoint blueCrossVerticalLeftOutside = WMPoint(-122.5,5);
    WMPoint blueCrossVerticalLeftInside = WMPoint(-117.5,5);
    WMPoint blueCrossVerticalRightOutside = WMPoint(-122.5,-5);
    WMPoint blueCrossVerticalRightInside = WMPoint(-117.5,-5);

    //Centre Cross Points
    WMPoint centreCrossYellowLeft = WMPoint(5,2.5);
    WMPoint centreCrossYellowRight = WMPoint(5,-2.5);
    WMPoint centreCrossBlueLeft = WMPoint(-5,2.5);
    WMPoint centreCrossBlueRight = WMPoint(-5,-2.5);

    static const double DTOR = 0.017543859; //Degree to radian ratio

    //Inner Centre Circle Points. 16 in total, numbered anticlockwise from x axis.

    WMPoint centreCircleInner1 = WMPoint(57.5,0);
    WMPoint centreCircleInner2 = WMPoint(57.5 * cos(27.5*DTOR),57.5 * sin(27.5*DTOR));
    WMPoint centreCircleInner3 = WMPoint(57.5 * cos(45*DTOR),57.5 * sin(45*DTOR));
    WMPoint centreCircleInner4 = WMPoint(57.5 * cos(72.5*DTOR),57.5 * sin(72.5*DTOR));
    WMPoint centreCircleInner5 = WMPoint(0,57.5);
    WMPoint centreCircleInner6 = WMPoint(-57.5 * cos(72.5*DTOR),57.5 * sin(72.5*DTOR));
    WMPoint centreCircleInner7 = WMPoint(-57.5 * cos(45*DTOR),57.5 * sin(45*DTOR));
    WMPoint centreCircleInner8 = WMPoint(-57.5 * cos(27.5*DTOR),57.5 * sin(27.5*DTOR));
    WMPoint centreCircleInner9 = WMPoint(-57.5,0);
    WMPoint centreCircleInner10 = WMPoint(-57.5 * cos(27.7*DTOR),-57.5 * sin(27.5*DTOR));
    WMPoint centreCircleInner11 = WMPoint(-57.5 * cos(45*DTOR),-57.5 * sin(45*DTOR));
    WMPoint centreCircleInner12 = WMPoint(-57.5 * cos(72.5*DTOR),-57.5 * sin(72.5*DTOR));
    WMPoint centreCircleInner13 = WMPoint(0,-57.5);
    WMPoint centreCircleInner14 = WMPoint(57.5 * cos(72.5*DTOR),-57.5 * sin(72.5*DTOR));
    WMPoint centreCircleInner15 = WMPoint(57.5 * cos(45*DTOR),-57.5 * sin(45*DTOR));
    WMPoint centreCircleInner16 = WMPoint(57.5 * cos(27.5*DTOR),-57.5 * sin(27.5*DTOR));

    //Outer Centre Circle Points.  Arranged as per inner.
    WMPoint centreCircleOuter1 = WMPoint(62.5,0);
    WMPoint centreCircleOuter2 = WMPoint(62.5 * cos(27.5*DTOR),62.5 * sin(27.5*DTOR));
    WMPoint centreCircleOuter3 = WMPoint(62.5 * cos(45*DTOR),62.5 * sin(45*DTOR));
    WMPoint centreCircleOuter4 = WMPoint(62.5 * cos(72.5*DTOR),62.5 * sin(72.5*DTOR));
    WMPoint centreCircleOuter5 = WMPoint(0,62.5);
    WMPoint centreCircleOuter6 = WMPoint(-62.5 * cos(72.5*DTOR),62.5 * sin(72.5*DTOR));
    WMPoint centreCircleOuter7 = WMPoint(-62.5 * cos(45*DTOR),62.5 * sin(45*DTOR));
    WMPoint centreCircleOuter8 = WMPoint(-62.5 * cos(27.5*DTOR),62.5 * sin(27.5*DTOR));
    WMPoint centreCircleOuter9 = WMPoint(-62.5,0);
    WMPoint centreCircleOuter10 = WMPoint(-62.5 * cos(27.7*DTOR),-62.5 * sin(27.5*DTOR));
    WMPoint centreCircleOuter11 = WMPoint(-62.5 * cos(45*DTOR),-62.5 * sin(45*DTOR));
    WMPoint centreCircleOuter12 = WMPoint(-62.5 * cos(72.5*DTOR),-62.5 * sin(72.5*DTOR));
    WMPoint centreCircleOuter13 = WMPoint(0,-62.5);
    WMPoint centreCircleOuter14 = WMPoint(62.5 * cos(72.5*DTOR),-62.5 * sin(72.5*DTOR));
    WMPoint centreCircleOuter15 = WMPoint(62.5 * cos(45*DTOR),-62.5 * sin(45*DTOR));
    WMPoint centreCircleOuter16 = WMPoint(62.5 * cos(27.5*DTOR),-62.5 * sin(27.5*DTOR));

    //Lines

    //Yellow Goal Line
    WMLine goalWMLineYellowOuter = WMLine(cornerYellowLeftOuter,cornerYellowRightOuter);
    WMLine goalWMLineYellowInner = WMLine(cornerYellowLeftInner,cornerYellowRightInner);

    line_array[0] = goalWMLineYellowOuter;
    line_array[1] = goalWMLineYellowInner;

    //Yellow Penalty Box Sides
    WMLine boxYellowLeftOuter = WMLine(goalWMLineYellowLeftOuter,penaltyYellowLeftOuter);
    WMLine boxYellowLeftInner = WMLine(goalWMLineYellowLeftInner,penaltyYellowLeftInner);
    WMLine boxYellowRightOuter = WMLine(goalWMLineYellowRightOuter,penaltyYellowRightOuter);
    WMLine boxYellowRightInner = WMLine(goalWMLineYellowRightInner,penaltyYellowRightInner);

    line_array[2] = boxYellowLeftOuter;
    line_array[3] = boxYellowLeftInner;
    line_array[4] = boxYellowRightOuter;
    line_array[5] = boxYellowRightInner;

    //Yellow Penalty Box Front
    WMLine boxYellowFrontOuter = WMLine(penaltyYellowLeftOuter,penaltyYellowRightOuter);
    WMLine boxYellowFrontInner = WMLine(penaltyYellowLeftInner,penaltyYellowRightInner);

    line_array[6] = boxYellowFrontOuter;
    line_array[7] = boxYellowFrontInner;

    //Side Lines
    WMLine sideLeftOuter = WMLine(cornerYellowLeftOuter,cornerBlueLeftOuter);
    WMLine sideLeftInner = WMLine(cornerYellowLeftInner,cornerBlueLeftInner);
    WMLine sideRightOuter = WMLine(cornerYellowRightOuter,cornerBlueRightOuter);
    WMLine sideRightInner = WMLine(cornerYellowRightInner,cornerBlueRightInner);

    line_array[8] = sideLeftOuter;
    line_array[9] = sideLeftInner;
    line_array[10] = sideRightOuter;
    line_array[11] = sideRightInner;

    //Centre Line
    WMLine centreYellow = WMLine(sideYellowLeft,sideYellowRight);
    WMLine centreBlue = WMLine(sideBlueLeft,sideBlueRight);

    line_array[12] = centreYellow;
    line_array[13] = centreBlue;

    //Blue Goal Line
    WMLine goalWMLineBlueOuter = WMLine(cornerBlueLeftOuter,cornerBlueRightOuter);
    WMLine goalWMLineBlueInner = WMLine(cornerBlueLeftInner,cornerBlueRightInner);

    line_array[14] = goalWMLineBlueOuter;
    line_array[15] = goalWMLineBlueInner;

    //Blue Penalty Box Sides
    WMLine boxBlueLeftOuter = WMLine(goalWMLineBlueLeftOuter,penaltyBlueLeftOuter);
    WMLine boxBlueLeftInner = WMLine(goalWMLineBlueLeftInner,penaltyBlueLeftInner);
    WMLine boxBlueRightOuter = WMLine(goalWMLineBlueRightOuter,penaltyBlueRightOuter);
    WMLine boxBlueRightInner = WMLine(goalWMLineBlueRightInner,penaltyBlueRightInner);

    line_array[16] = boxBlueLeftOuter;
    line_array[17] = boxBlueLeftInner;
    line_array[18] = boxBlueRightOuter;
    line_array[19] = boxBlueRightInner;

    //Blue Penalty Box Front
    WMLine boxBlueFrontOuter = WMLine(penaltyBlueLeftOuter,penaltyBlueRightOuter);
    WMLine boxBlueFrontInner = WMLine(penaltyBlueLeftInner,penaltyBlueRightInner);

    line_array[20] = boxBlueFrontOuter;
    line_array[21] = boxBlueFrontInner;

    //Yellow Cross
    WMLine crossYellowHorizontalLeft = WMLine(yellowCrossHorizontalOutsideLeft,yellowCrossHorizontalInsideLeft);
    WMLine crossYellowHorizontalRight = WMLine(yellowCrossHorizontalOutsideRight,yellowCrossHorizontalInsideRight);
    WMLine crossYellowVerticalTop = WMLine(yellowCrossVerticalLeftOutside,yellowCrossVerticalRightOutside);
    WMLine crossYellowVerticalBottom = WMLine(yellowCrossVerticalRightInside,yellowCrossVerticalLeftInside);
    WMLine crossYellowHorizontalTop = WMLine(yellowCrossHorizontalOutsideLeft,yellowCrossHorizontalOutsideRight);
    WMLine crossYellowHorizontalBottom = WMLine(yellowCrossHorizontalInsideLeft,yellowCrossHorizontalInsideRight);
    WMLine crossYellowVerticalLeft = WMLine(yellowCrossVerticalLeftOutside,yellowCrossVerticalLeftInside);
    WMLine crossYellowVerticalRight = WMLine(yellowCrossVerticalRightOutside,yellowCrossVerticalRightInside);

    line_array[22] = crossYellowHorizontalLeft;
    line_array[23] = crossYellowHorizontalRight;
    line_array[24] = crossYellowVerticalTop;
    line_array[25] = crossYellowVerticalBottom;
    line_array[26] = crossYellowHorizontalTop;
    line_array[27] = crossYellowHorizontalBottom;
    line_array[28] = crossYellowVerticalLeft;
    line_array[29] = crossYellowVerticalRight;

    //Blue Cross
    WMLine crossBlueHorizontalLeft = WMLine(blueCrossHorizontalOutsideLeft,blueCrossHorizontalInsideLeft);
    WMLine crossBlueHorizontalRight = WMLine(blueCrossHorizontalOutsideRight,blueCrossHorizontalInsideRight);
    WMLine crossBlueVerticalTop = WMLine(blueCrossVerticalLeftOutside,blueCrossVerticalRightOutside);
    WMLine crossBlueVerticalBottom = WMLine(blueCrossVerticalRightInside,blueCrossVerticalLeftInside);
    WMLine crossBlueHorizontalTop = WMLine(blueCrossHorizontalOutsideLeft,blueCrossHorizontalOutsideRight);
    WMLine crossBlueHorizontalBottom = WMLine(blueCrossHorizontalInsideLeft,blueCrossHorizontalInsideRight);
    WMLine crossBlueVerticalLeft = WMLine(blueCrossVerticalLeftOutside,blueCrossVerticalLeftInside);
    WMLine crossBlueVerticalRight = WMLine(blueCrossVerticalRightOutside,blueCrossVerticalRightInside);

    line_array[30] = crossBlueHorizontalLeft;
    line_array[31] = crossBlueHorizontalRight;
    line_array[32] = crossBlueVerticalTop;
    line_array[33] = crossBlueVerticalBottom;
    line_array[34] = crossBlueHorizontalTop;
    line_array[35] = crossBlueHorizontalBottom;
    line_array[36] = crossBlueVerticalLeft;
    line_array[37] = crossBlueVerticalRight;

    WMLine crossCentreLeft = WMLine(centreCrossYellowLeft,centreCrossBlueLeft);
    WMLine crossCentreRight = WMLine(centreCrossYellowRight,centreCrossBlueRight);
    WMLine crossCentreYellow = WMLine(centreCrossYellowLeft,centreCrossYellowRight);
    WMLine crossCentreBlue = WMLine(centreCrossBlueLeft,centreCrossBlueRight);

    line_array[38] = crossCentreLeft;
    line_array[39] = crossCentreRight;
    line_array[40] = crossCentreYellow;
    line_array[41] = crossCentreBlue;

    //Centre Circle Inner Lines
    WMLine circleInner1 = WMLine(centreCircleInner1,centreCircleInner2);
    WMLine circleInner2 = WMLine(centreCircleInner2,centreCircleInner3);
    WMLine circleInner3 = WMLine(centreCircleInner3,centreCircleInner4);
    WMLine circleInner4 = WMLine(centreCircleInner4,centreCircleInner5);
    WMLine circleInner5 = WMLine(centreCircleInner5,centreCircleInner6);
    WMLine circleInner6 = WMLine(centreCircleInner6,centreCircleInner7);
    WMLine circleInner7 = WMLine(centreCircleInner7,centreCircleInner8);
    WMLine circleInner8 = WMLine(centreCircleInner8,centreCircleInner9);
    WMLine circleInner9 = WMLine(centreCircleInner9,centreCircleInner10);
    WMLine circleInner10 = WMLine(centreCircleInner10,centreCircleInner11);
    WMLine circleInner11 = WMLine(centreCircleInner11,centreCircleInner12);
    WMLine circleInner12 = WMLine(centreCircleInner12,centreCircleInner13);
    WMLine circleInner13 = WMLine(centreCircleInner13,centreCircleInner14);
    WMLine circleInner14 = WMLine(centreCircleInner14,centreCircleInner15);
    WMLine circleInner15 = WMLine(centreCircleInner15,centreCircleInner16);
    WMLine circleInner16 = WMLine(centreCircleInner16,centreCircleInner1);

    line_array[42] = circleInner1;
    line_array[43] = circleInner2;
    line_array[44] = circleInner3;
    line_array[45] = circleInner4;
    line_array[46] = circleInner5;
    line_array[47] = circleInner6;
    line_array[48] = circleInner7;
    line_array[49] = circleInner8;
    line_array[50] = circleInner9;
    line_array[51] = circleInner10;
    line_array[52] = circleInner11;
    line_array[53] = circleInner12;
    line_array[54] = circleInner13;
    line_array[55] = circleInner14;
    line_array[56] = circleInner15;
    line_array[57] = circleInner16;

    //Centre Circle Outer Lines
    WMLine circleOuter1 = WMLine(centreCircleOuter1,centreCircleOuter2);
    WMLine circleOuter2 = WMLine(centreCircleOuter2,centreCircleOuter3);
    WMLine circleOuter3 = WMLine(centreCircleOuter3,centreCircleOuter4);
    WMLine circleOuter4 = WMLine(centreCircleOuter4,centreCircleOuter5);
    WMLine circleOuter5 = WMLine(centreCircleOuter5,centreCircleOuter6);
    WMLine circleOuter6 = WMLine(centreCircleOuter6,centreCircleOuter7);
    WMLine circleOuter7 = WMLine(centreCircleOuter7,centreCircleOuter8);
    WMLine circleOuter8 = WMLine(centreCircleOuter8,centreCircleOuter9);
    WMLine circleOuter9 = WMLine(centreCircleOuter9,centreCircleOuter10);
    WMLine circleOuter10 = WMLine(centreCircleOuter10,centreCircleOuter11);
    WMLine circleOuter11 = WMLine(centreCircleOuter11,centreCircleOuter12);
    WMLine circleOuter12 = WMLine(centreCircleOuter12,centreCircleOuter13);
    WMLine circleOuter13 = WMLine(centreCircleOuter13,centreCircleOuter14);
    WMLine circleOuter14 = WMLine(centreCircleOuter14,centreCircleOuter15);
    WMLine circleOuter15 = WMLine(centreCircleOuter15,centreCircleOuter16);
    WMLine circleOuter16 = WMLine(centreCircleOuter16,centreCircleOuter1);

    line_array[58] = circleOuter1;
    line_array[59] = circleOuter2;
    line_array[60] = circleOuter3;
    line_array[61] = circleOuter4;
    line_array[62] = circleOuter5;
    line_array[63] = circleOuter6;
    line_array[64] = circleOuter7;
    line_array[65] = circleOuter8;
    line_array[66] = circleOuter9;
    line_array[67] = circleOuter10;
    line_array[68] = circleOuter11;
    line_array[69] = circleOuter12;
    line_array[70] = circleOuter13;
    line_array[71] = circleOuter14;
    line_array[72] = circleOuter15;
    line_array[73] = circleOuter16;

}
void LocalisationWidget::createGoals()
{
    //Points

    WMPoint yellowLeftGoalBase = WMPoint(300,70,0);
    WMPoint yellowRightGoalBase = WMPoint(300,-70,0);

    WMPoint yellowLeftGoalTop = WMPoint(300,70,80);
    WMPoint yellowRightGoalTop = WMPoint(300,-70,80);

    WMPoint blueLeftGoalBase = WMPoint(-300,70,0);
    WMPoint blueRightGoalBase = WMPoint(-300,-70,0);

    WMPoint blueLeftGoalTop = WMPoint(-300,70,80);
    WMPoint blueRightGoalTop = WMPoint(-300,-70,80);

    //Goals

    goal_array[0].setMid(yellowLeftGoalBase,yellowLeftGoalTop);
    goal_array[1].setMid(yellowRightGoalBase,yellowRightGoalTop);
    goal_array[2].setMid(blueRightGoalBase,blueRightGoalTop);
    goal_array[3].setMid(blueLeftGoalBase,blueLeftGoalTop);
}
