/*!
  @file localisationwidget.h
  @author Jarrod West.
  @brief Declaration and implementation of class LocalisationWidget.
  */

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

/*!
  @brief A class that calculates a robot's visual space based on its joint and position information.
  */

class LocalisationWidget : public QDockWidget
{
    Q_OBJECT
public:
    /*!
      @brief A constructor for LocalisationWidget class.
      @param parent The QWidget parent of this widget.
      */
    LocalisationWidget(QWidget *parent = 0);

    /*!
      @brief A destructor for LocalisationWidget class.
      */
    ~LocalisationWidget();

    /*!
      @brief Function to create the graphic widgets for the class.
      */
    void createWidgets();

    /*!
      @brief Function to create the graphic layout of the class.
      */
    void createLayout();

    /*!
      @brief Function to create the signal and slot connections for the class.
      */
    void createConnections();

    /*!
      @brief Function to create the world model lines.
      */
    void createLines();

    /*!
      @brief Function to create the world model goals.
      */
    void createGoals();

    /*!
      @brief Calculates if a world model line is visible and sends it to the openglmanager to be drawn.
      @param newLine The line to be displayed.
      */
    void displayLines(WMLine newLine);

    /*!
      @brief Calculates if a world model goal is visible and sends it to the openglmanager to be drawn.
      @param newCylinder The goal to be displayed.
      */
    void displayGoals(Cylinder newCylinder);

    /*!
      @brief Calculates if a world model ball is visible and sends it to the openglmanager to be drawn.
      @param newSphere The ball to be displayed.
      */
    void displayBall(Sphere newSphere);

    /*!
      @brief Rounds a decimal number to the nearest integer.
      @param newInt The double to be rounded.
      */
    int roundDouble(double newInt);

    /*!
      @brief Calculates the total, pitch and yaw angle for the robot to look directly at a given point.
      @param point The point to calculate to.
      */
    void angleToPoint(WMPoint point);

    /*!
      @brief Converts a double to a QString.
      @param newString The double to be converted.
      */
    QString doubleToString(double newString);

    /*!
      @brief Updates the visible joint data on the graphic widget.
      */
    void updateJoints();

    /*!
      @brief Calculates the visible lines whenever the frame is updated.
      */
    void calculateLines();

public slots:
    /*!
      @brief Accepts a new value for xInput and updates the visible lines.
      @param newX The new value for xInput.
      */
    void xChanged(double newX);

    /*!
      @brief Accepts a new value for yInput and updates the visible lines.
      @param newY The new value for yInput.
      */
    void yChanged(double newY);

    /*!
      @brief Accepts a new value for thetaInput and updates the visible lines.
      @param newTheta The new value for thetaInput.
      */
    void thetaChanged(double newTheta);

    /*!
      @brief Accepts a new value for ballX and updates the visible lines.
      @param newBallX The new value for ballX.
      */
    void ballXChanged(double newBallX);

    /*!
      @brief Accepts a new value for ballY and updates the visible lines.
      @param newBallY The new value for ballY.
      */
    void ballYChanged(double newBallY);

    /*!
      @brief Accepts a new value for the robot's head yaw paramater and updates the visible lines.
      @param newHeadYaw The new head yaw paramater value.
      */
    void headYawChanged(double newHeadYaw);

    /*!
      @brief Accepts a new value for the robot's head pitch paramater and updates the visible lines.
      @param newHeadPitch The new head pitch paramater value.
      */
    void headPitchChanged(double newHeadPitch);

    /*!
      @brief Accepts a new value for the robot's left hip pitch paramater and updates the visible lines.
      @param newLHipPitch The new left hip pitch paramater value.
      */
    void lHipPitchChanged(double newLHipPitch);

    /*!
      @brief Accepts a new value for the robot's left hip roll paramater and updates the visible lines.
      @param newLHipRoll The new left hip roll paramater value.
      */
    void lHipRollChanged(double newLHipRoll);

    /*!
      @brief Accepts a new value for the robot's right hip pitch paramater and updates the visible lines.
      @param newRHipPitch The new right hip pitch paramater value.
      */
    void rHipPitchChanged(double newRHipPitch);

    /*!
      @brief Accepts a new value for the robot's right hip roll paramater and updates the visible lines.
      @param newRHipRoll The new right hip roll paramater value.
      */
    void rHipRollChanged(double newRHipRoll);

    /*!
      @brief Accepts the robot's sensor data each change of frame and updates the local variables.
      @param jointSensors A pointer to the start of an array of doubles containing the joint sensor data.
      @param camera A boolean representing the camera that is being used.
      @param touchSensors A pointer to the start of an array of doubles containing the touch sensor data.
      */
    void frameChange(const double* jointSensors,bool camera,const double* touchSensors);

    /*!
      @brief Accepts a new value for the x ordinate of anglePoint.
      @param newAngleX The new x ordinate of anglePoint.
      */
    void angleXChanged(double newAngleX);

    /*!
      @brief Accepts a new value for the y ordinate of anglePoint.
      @param newAngleY The new y ordinate of anglePoint.
      */
    void angleYChanged(double newAngleY);

    /*!
      @brief Accepts a new value for the z ordinate of anglePoint.
      @param newAngleZ The new z ordinate of anglePoint.
      */
    void angleZChanged(double newAngleZ);

    /*!
      @brief Function to call angleToPoint when the calculateAngle button is clicked.
      */
    void calculateAngleClicked();

    /*!
      @brief Function to update anglePoint with the result of the commonPoints combo box.
      @param newIndex The index of the new point.
    */
    void commonPointsChanged(int newIndex);

signals:
    /*!
      @brief Updates a display with the world model lines.
      @param lines A pointer to the start of the array of lines to be displayed.
      @param numberOfLines The number of lines to be displayed.
      @param displayID The id of the display being updated.
      */
    void updateLocalisationLine(WMLine* lines,int numberOfLines,GLDisplay::display displayID);

    /*!
      @brief Updates a display with the world model lines.
      @param bx The x ordinate of the ball to be displayed.
      @param by The y ordinate of the ball to be displayed.
      @param brad The radius of the ball to be displayed.
      @param displayID The id of the display being updated.
      */
    void updateLocalisationBall(float bx,float by,float brad,GLDisplay::display displayID);

    /*!
      @brief Clears a given display.
      @param displayID The id of the display being cleared.
      */
    void removeLocalisationLine(GLDisplay::display displayID);

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


    double paramaters[6];                   //!< The joint angle paramaters.
    WMLine line_array[74];                  //!< The world model lines.
    Cylinder goal_array[4];                 //!< The world model goals.
    Sphere ball;                            //!< The world model ball.

    CameraMatrix camera;                    //!< The camera matrix

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
