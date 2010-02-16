#ifndef LOCALISATIONWIDGET_H
#define LOCALISATIONWIDGET_H

#include <QWidget>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QDockWidget>
#include <QGridLayout>
#include <QGroupBox>
#include <QCheckBox>
#include <QComboBox>
#include <QPushButton>
#include "GLDisplay.h"
#include "Localisation/cameramatrix.h"
#include "Localisation/cylinder.h"
#include "Localisation/sphere.h"

/*QT_BEGIN_NAMESPACE
class QCheckBox;
//class QHBoxLayout;
class QDoubleSpinBox;
class QGroupBox;
//class QComboBox;
QT_END_NAMESPACE*/

class LocalisationWidget : public QDockWidget
{
    Q_OBJECT
public:
    LocalisationWidget(QWidget *parent = 0);
    ~LocalisationWidget();
    void createWidgets();
    void createLayout();
    void createConnections();

    void createLines();
    void createGoals();

    void displayLines(WMLine in);
    void displayGoals(Cylinder in);
    void displayBall(Sphere in);
    int roundDouble(double in);
    //double distance(WMPoint p1, WMPoint p2);
    void angleToPoint(WMPoint p);
    QString doubleToString(double in);
    void updateJoints();

    void calculateLines();

public slots:
    void xChanged(double newX);
    void yChanged(double newY);
    void thetaChanged(double newTheta);
    void ballXChanged(double newBallX);
    void ballYChanged(double newBallY);

    void headYawChanged(double newLHeadYaw);
    void headPitchChanged(double newLHeadPitch);
    void lHipPitchChanged(double newLHipPitch);
    void lHipRollChanged(double newLHipRoll);
    void rHipPitchChanged(double newRHipPitch);
    void rHipRollChanged(double newRHipRoll);

    void frameChange(double* jointSensors,bool camera,double* touchSensors);
    void angleXChanged(double newAngleX);
    void angleYChanged(double newAngleY);
    void angleZChanged(double newAngleZ);

    void calculateAngleClicked();
    void commonPointsChanged(int newIndex);

signals:
    void updateLocalisationLine(WMLine*,int,GLDisplay::display);
    void updateLocalisationBall(float,float,float,GLDisplay::display);
    void removeLocalisationLine(GLDisplay::display);

private:
    QDoubleSpinBox *xSpinBox,*ySpinBox,*thetaSpinBox,*ballXSpinBox,*ballYSpinBox;

    QDoubleSpinBox *lHipPitchSpinBox,*lHipRollSpinBox,*headPitchSpinBox,*headYawSpinBox;
    QDoubleSpinBox *rHipPitchSpinBox,*rHipRollSpinBox;

    QDoubleSpinBox *anglePointXSpinBox,*anglePointYSpinBox,*anglePointZSpinBox;

    QLabel *xLabel,*yLabel,*thetaLabel,*ballXLabel,*ballYLabel;
    QLabel *lHipPitchLabel,*lHipRollLabel,*headPitchLabel,*headYawLabel,*rHipPitchLabel,*rHipRollLabel, *setLocalisationLabel;

    QLabel *anglePointXLabel,*anglePointYLabel,*anglePointZLabel;
    QLabel * totalAngleLabel,*yawAngleLabel,*pitchAngleLabel,*totalAngle,*yawAngle,*pitchAngle;

    QLabel *headYawJointLabel,*headPitchJointLabel,*lShoulderRollJointLabel,*lShoulderPitchJointLabel;
    QLabel *lElbowYawJointLabel,*lElbowRollJointLabel,*rShoulderRollJointLabel,*rShoulderPitchJointLabel;
    QLabel *rElbowYawJointLabel,*rElbowRollJointLabel,*lHipYawPitchJointLabel,*lHipRollJointLabel;
    QLabel *lHipPitchJointLabel, *lKneePitchJointLabel, *lAnklePitchJointLabel,*lAnkleRollJointLabel;
    QLabel *rHipYawPitchJointLabel,*rHipRollJointLabel,*rHipPitchJointLabel, *rKneePitchJointLabel;
    QLabel *rAnklePitchJointLabel,*rAnkleRollJointLabel;
    QLabel *cameraLabel;

    QLabel *headYawText,*headPitchText,*lHipPitchYawText,*lHipRollText,*lHipPitchText,*lKneePitchText;
    QLabel *lAnklePitchText,*lAnkleRollText,*rHipPitchYawText,*rHipRollText,*rHipPitchText,*rKneePitchText;
    QLabel *rAnklePitchText,*rAnkleRollText;
    QLabel *cameraText;

    QGridLayout* localisationLayout, *paramaterLayout,*angleToPointLayout,*jointsLayout;
    QGridLayout *groupLayout,*totalLayout;
    QGroupBox *localisationGroupBox,*paramaterGroupBox,*angleToPointGroupBox,*jointsGroupBox;
    QComboBox *commonPointsComboBox;
    QPushButton *calculateAngle;
    QWidget* window;
    double paramaters[6];
    WMLine line_array[74];
    Cylinder goal_array[4];
    Sphere ball;
    int scale;

    CameraMatrix camera;

    double ballX, ballY;
    double xInput,yInput,thetaInput;
    double joints[22];
    bool bottomCamera;
    bool leftLegOnGround,rightLegOnGround;

    WMLine visibleLines[100];
    int countVisibleLines;

    WMPoint anglePoint;
};

#endif // LOCALISATIONWIDGET_H
