#ifndef CAMERASETTINGSWIDGET_H
#define CAMERASETTINGSWIDGET_H

#include <QWidget>
#include <QString>
#include <QDockWidget>

#include <QByteArray>
#include <stdio.h>
#include <iostream>
#include <QTimer>
class QMdiArea;
class QComboBox;
class QCheckBox;
class QLabel;
class QSlider;
class QSpinBox;
class QVBoxLayout;
class QHBoxLayout;
class QGridLayout;
class QMdiSubWindow;
class QPushButton;
class QCheckBox;
class QSignalMapper;
class QToolButton;
class QLineEdit;
class QTcpSocket;
class QGridLayout;
class QGroupBox;

class CameraSettings;
class JobList;


class cameraSettingsWidget: public QWidget
{
    Q_OBJECT

public:
    cameraSettingsWidget(QMdiArea* parentMdiWidget, QWidget *parent = 0);
    ~cameraSettingsWidget();

private slots:
    /*!
      @brief Reads the settings for the currently selected layer and sets
             the current selection to these.
      */
    void cameraSettingsChanged();
    void updateRobotName(const QString name);
    void getCameraSetting();
    void streamCameraSetting();
    void stopStreamCameraSetting();
    void readPendingData();
    void sendSettingsToRobot();
    void sendDataToRobot();\

    void sendStartSavingImagesJob();
    void sendStopSavingImagesJob();

private:
    QVBoxLayout* overallLayout;                 //!< Overall widget layout.

    // Shift Gain
    QHBoxLayout* shiftGainLayout;          //!< Layout for shift Gain
    QLabel* shiftGainLabel;                //!< Label for shift Gain
    QSlider* shiftGainSlider;              //!< Slider for Gain selection
    QSpinBox* shiftGainSpinBox;            //!< SpinBox for Gain selection

    // Shift Exposure
    QHBoxLayout* shiftExposureLayout;          //!< Layout for shift Exposure
    QLabel* shiftExposureLabel;                //!< Label for shift Exposure
    QSlider* shiftExposureSlider;              //!< Slider for Exposure selection
    QSpinBox* shiftExposureSpinBox;            //!< SpinBox for Exposure selection

    // Shift BlueChroma
    QHBoxLayout* shiftBlueChromaLayout;          //!< Layout for shift BlueChroma
    QLabel* shiftBlueChromaLabel;                //!< Label for shift BlueChroma
    QSlider* shiftBlueChromaSlider;              //!< Slider for BlueChroma selection
    QSpinBox* shiftBlueChromaSpinBox;            //!< SpinBox for BlueChroma selection

    // Shift RedChroma
    QHBoxLayout* shiftRedChromaLayout;          //!< Layout for shift RedChroma
    QLabel* shiftRedChromaLabel;                //!< Label for shift RedChroma
    QSlider* shiftRedChromaSlider;              //!< Slider for RedChroma selection
    QSpinBox* shiftRedChromaSpinBox;            //!< SpinBox for RedChroma selection

    // Shift Brightness
    QHBoxLayout* shiftBrightnessLayout;          //!< Layout for shift Brightness
    QLabel* shiftBrightnessLabel;                //!< Label for shift Brightness
    QSlider* shiftBrightnessSlider;              //!< Slider for Brightness selection
    QSpinBox* shiftBrightnessSpinBox;            //!< SpinBox for Brightness selection

    // Shift Saturation
    QHBoxLayout* shiftSaturationLayout;          //!< Layout for shift Saturation
    QLabel* shiftSaturationLabel;                //!< Label for shift Saturation
    QSlider* shiftSaturationSlider;              //!< Slider for Saturation selection
    QSpinBox* shiftSaturationSpinBox;            //!< SpinBox for Saturation selection

    // Shift Contrast
    QHBoxLayout* shiftContrastLayout;          //!< Layout for shift Contrast
    QLabel* shiftContrastLabel;                //!< Label for shift Contrast
    QSlider* shiftContrastSlider;              //!< Slider for Contrast selection
    QSpinBox* shiftContrastSpinBox;            //!< SpinBox for Contrast selection

    // Shift Hue
    QHBoxLayout* shiftHueLayout;          //!< Layout for shift Hue
    QLabel* shiftHueLabel;                //!< Label for shift Hue
    QSlider* shiftHueSlider;              //!< Slider for Hue selection
    QSpinBox* shiftHueSpinBox;            //!< SpinBox for Hue selection

    // Auto Settings
    QGridLayout* AutoTickBoxLayout;
    QGroupBox* AutoCameraSettingsGroupBox;

    QLabel* ActiveCamera;                   //!< Label for Camera
    QCheckBox* TopCameraSelected;
    QCheckBox* BottomCameraSelected;
    QCheckBox* AutoGainSelected;
    QCheckBox* AutoWhiteBalanceSelected;
    QCheckBox* AutoExposureSelected;

    QPushButton* getCameraSettingsButton;
    QPushButton* streamCameraSettingsButton;
    QPushButton* stopStreamCameraSettingsButton;

    QGroupBox* CameraSettingsGroupBox;
    QGridLayout* SelectCameraLayout;
    QGroupBox* SelectCameraGroupBox;
    QGridLayout* SliderLayout;
    QHBoxLayout* robotNameInputLayout;
    QHBoxLayout* pushButtonLayout;
    QHBoxLayout* saveImagesButtonLayout;
    QString robotName;

    QPushButton* StartSavingImagesButton;
    QPushButton* StopSavingImagesButton;
    QCheckBox* StartSavingImagesWithSettingsCheckBox;


    int datasize;
    QLabel* nameLabel;
    QLineEdit* nameLineEdit;

    CameraSettings* settings;
    JobList* m_job_list;

    void connectToRobot();
    void disconnectFromRobot();
    void createWidgets();       //!< Create all of the child widgets.
    void createLayout();        //!< Layout all of the child widgets.
    void createConnections();   //!< Connect all of the child widgets.
    bool disableWriting;        //!< Flag used to disable the writing of settings back to the layers when updating the displays.
    //NETWORKING:
    QTcpSocket* tcpSocket;
    QByteArray netdata;
    QTimer timer;
    QTimer readPacketTimer;
    bool dostream;

};

#endif // CAMERASETTINGSWIDGET_H
