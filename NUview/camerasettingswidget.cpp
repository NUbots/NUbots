#include "camerasettingswidget.h"
// Qt Includes
#include <QMdiArea>
#include <QComboBox>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QMdiSubWindow>
#include <QPixmap>
#include <QPushButton>
#include <QSignalMapper>
#include <QPainter>
#include <QToolButton>
#include <QColorDialog>
#include <QLineEdit>
#include <QTcpSocket>
#include <sstream>


#include "NUviewIO/NUviewIO.h"
#include "Behaviour/Jobs.h"
#include "Behaviour/Jobs/CameraJobs/ChangeCameraSettingsJob.h"
#include "Behaviour/Jobs/VisionJobs/SaveImagesJob.h"
#include "NUPlatform/NUCamera/CameraSettings.h"
#include "debug.h"


cameraSettingsWidget::cameraSettingsWidget(QMdiArea* parentMdiWidget, QWidget *parent): QWidget(parent)
{
    setObjectName(tr("Camera Settings"));
    setWindowTitle(tr("Camera Settings"));
    tcpSocket = new QTcpSocket();
    robotName = "";
    timer.setInterval(200);
    readPacketTimer.setInterval(100);
    createWidgets();
    createLayout();
    createConnections();
    settings = new CameraSettings();
    m_job_list = new JobList();
    this->setEnabled(true);
}

cameraSettingsWidget::~cameraSettingsWidget()
{
    delete settings;
    delete m_job_list;

}

void cameraSettingsWidget::createWidgets()  //!< Create all of the child widgets.
{
    // Shift Gain
    shiftGainLabel = new QLabel("Gain:");                //!< Label for shift Gain
    shiftGainSlider = new QSlider(Qt::Horizontal);       //!< Slider for Gain selection
    shiftGainSlider->setMinimum(0);
    shiftGainSlider->setMaximum(255);

    shiftGainSpinBox = new QSpinBox();
    shiftGainSpinBox->setMinimum(shiftGainSlider->minimum());
    shiftGainSpinBox->setMaximum(shiftGainSlider->maximum());


    // Shift Exposure
    shiftExposureLabel = new QLabel("Exposure:");                //!< Label for shift Exposure
    shiftExposureSlider = new QSlider(Qt::Horizontal);       //!< Slider for Exposure selection
    shiftExposureSlider->setMinimum(0);
    shiftExposureSlider->setMaximum(510);

    shiftExposureSpinBox = new QSpinBox();
    shiftExposureSpinBox->setMinimum(shiftExposureSlider->minimum());
    shiftExposureSpinBox->setMaximum(shiftExposureSlider->maximum());

    // Shift BlueChroma
    shiftBlueChromaLabel = new QLabel("Blue Chroma:");                //!< Label for shift BlueChroma
    shiftBlueChromaSlider = new QSlider(Qt::Horizontal);       //!< Slider for BlueChroma selection
    shiftBlueChromaSlider->setMinimum(0);
    shiftBlueChromaSlider->setMaximum(255);

    shiftBlueChromaSpinBox = new QSpinBox();
    shiftBlueChromaSpinBox->setMinimum(shiftBlueChromaSlider->minimum());
    shiftBlueChromaSpinBox->setMaximum(shiftBlueChromaSlider->maximum());


    // Shift RedChroma
    shiftRedChromaLabel = new QLabel("Red Chroma:");                //!< Label for shift BlueChroma
    shiftRedChromaSlider = new QSlider(Qt::Horizontal);       //!< Slider for BlueChroma selection
    shiftRedChromaSlider->setMinimum(0);
    shiftRedChromaSlider->setMaximum(255);

    shiftRedChromaSpinBox = new QSpinBox();
    shiftRedChromaSpinBox->setMinimum(shiftRedChromaSlider->minimum());
    shiftRedChromaSpinBox->setMaximum(shiftRedChromaSlider->maximum());

    // Shift Brightness
    shiftBrightnessLabel = new QLabel("Brightness:");                //!< Label for shift BlueChroma
    shiftBrightnessSlider = new QSlider(Qt::Horizontal);       //!< Slider for BlueChroma selection
    shiftBrightnessSlider->setMinimum(0);
    shiftBrightnessSlider->setMaximum(255);

    shiftBrightnessSpinBox = new QSpinBox();
    shiftBrightnessSpinBox->setMinimum(shiftBrightnessSlider->minimum());
    shiftBrightnessSpinBox->setMaximum(shiftBrightnessSlider->maximum());


    // Shift Saturation
    shiftSaturationLabel = new QLabel("Saturation:");                //!< Label for shift BlueChroma
    shiftSaturationSlider = new QSlider(Qt::Horizontal);       //!< Slider for BlueChroma selection
    shiftSaturationSlider->setMinimum(0);
    shiftSaturationSlider->setMaximum(255);

    shiftSaturationSpinBox = new QSpinBox();
    shiftSaturationSpinBox->setMinimum(shiftSaturationSlider->minimum());
    shiftSaturationSpinBox->setMaximum(shiftSaturationSlider->maximum());


    // Shift Contrast
    shiftContrastLabel = new QLabel("Contrast:");                //!< Label for shift BlueChroma
    shiftContrastSlider = new QSlider(Qt::Horizontal);       //!< Slider for BlueChroma selection
    shiftContrastSlider->setMinimum(0);
    shiftContrastSlider->setMaximum(127);

    shiftContrastSpinBox = new QSpinBox();
    shiftContrastSpinBox->setMinimum(shiftContrastSlider->minimum());
    shiftContrastSpinBox->setMaximum(shiftContrastSlider->maximum());

    // Shift Hue
    shiftHueLabel = new QLabel("Hue:");                //!< Label for shift BlueChroma
    shiftHueSlider = new QSlider(Qt::Horizontal);       //!< Slider for BlueChroma selection
    shiftHueSlider->setMinimum(-180);
    shiftHueSlider->setMaximum(180);

    shiftHueSpinBox = new QSpinBox();
    shiftHueSpinBox->setMinimum(shiftHueSlider->minimum());
    shiftHueSpinBox->setMaximum(shiftHueSlider->maximum());

    //Connection Stuff:
    nameLabel = new QLabel("Robot Name: ");
    nameLineEdit = new QLineEdit();
    nameLineEdit->setText("IP ADDRESS");
    streamCameraSettingsButton = new QPushButton("Start Stream");
    getCameraSettingsButton = new QPushButton("Get Current Settings");
    stopStreamCameraSettingsButton = new QPushButton("Stop Stream");
    getCameraSettingsButton->setEnabled(true);
    streamCameraSettingsButton->setEnabled(false);
    stopStreamCameraSettingsButton->setEnabled(false);

    // Save images
    StartSavingImagesButton = new QPushButton("Start Saving Images");
    StopSavingImagesButton = new QPushButton("Stop Saving Images");
    StartSavingImagesWithSettingsCheckBox = new QCheckBox("Vary Camera Settings");

}
void cameraSettingsWidget::createLayout()   //!< Layout all of the child widgets.
{
    // Shift Gain
    shiftGainLayout = new QHBoxLayout();                      //!< Layout for shift Gain
    shiftGainLayout->addWidget(shiftGainLabel);               //!< Label for shift Gain
    shiftGainLayout->addWidget(shiftGainSlider);              //!< Slider for Gain selection
    shiftGainLayout->addWidget(shiftGainSpinBox);             //!< SpinBox for Gain selection

    // Shift Exposure
    shiftExposureLayout = new QHBoxLayout();                        //!< Layout for shift Exposure
    shiftExposureLayout->addWidget(shiftExposureLabel);             //!< Label for shift Exposure
    shiftExposureLayout->addWidget(shiftExposureSlider);            //!< Slider for Exposure selection
    shiftExposureLayout->addWidget(shiftExposureSpinBox);           //!< SpinBox for Exposure selection


    // Shift BlueChroma
    shiftBlueChromaLayout = new QHBoxLayout();                          //!< Layout for shift BlueChroma
    shiftBlueChromaLayout->addWidget(shiftBlueChromaLabel);             //!< Label for shift BlueChroma
    shiftBlueChromaLayout->addWidget(shiftBlueChromaSlider);            //!< Slider for Exposure BlueChroma
    shiftBlueChromaLayout->addWidget(shiftBlueChromaSpinBox);           //!< SpinBox for Exposure BlueChroma

    // Shift RedChroma
    shiftRedChromaLayout = new QHBoxLayout();                          //!< Layout for shift RedChroma
    shiftRedChromaLayout->addWidget(shiftRedChromaLabel);             //!< Label for shift RedChroma
    shiftRedChromaLayout->addWidget(shiftRedChromaSlider);            //!< Slider for RedChroma selection
    shiftRedChromaLayout->addWidget(shiftRedChromaSpinBox);           //!< SpinBox for RedChroma selection

    // Shift Brightness
    shiftBrightnessLayout = new QHBoxLayout();                          //!< Layout for shift Brightness
    shiftBrightnessLayout->addWidget(shiftBrightnessLabel);             //!< Label for shift Brightness
    shiftBrightnessLayout->addWidget(shiftBrightnessSlider);            //!< Slider for Brightness selection
    shiftBrightnessLayout->addWidget(shiftBrightnessSpinBox);           //!< SpinBox for Brightness selection

    // Shift Saturation
    shiftSaturationLayout = new QHBoxLayout();                          //!< Layout for shift Saturation
    shiftSaturationLayout->addWidget(shiftSaturationLabel);             //!< Label for shift Saturation
    shiftSaturationLayout->addWidget(shiftSaturationSlider);            //!< Slider for Saturation selection
    shiftSaturationLayout->addWidget(shiftSaturationSpinBox);           //!< SpinBox for Saturation selection

    // Shift Contrast
    shiftContrastLayout = new QHBoxLayout();                          //!< Layout for shift Contrast
    shiftContrastLayout->addWidget(shiftContrastLabel);             //!< Label for shift Contrast
    shiftContrastLayout->addWidget(shiftContrastSlider);            //!< Slider for Contrast selection
    shiftContrastLayout->addWidget(shiftContrastSpinBox);           //!< SpinBox for Contrast selection

    // Shift Hue
    shiftHueLayout = new QHBoxLayout();                   //!< Layout for shift Hue
    shiftHueLayout->addWidget(shiftHueLabel);             //!< Label for shift Hue
    shiftHueLayout->addWidget(shiftHueSlider);            //!< Slider for Hue selection
    shiftHueLayout->addWidget(shiftHueSpinBox);           //!< SpinBox for Hue selection

    robotNameInputLayout = new QHBoxLayout();
    robotNameInputLayout->addWidget(nameLabel);
    robotNameInputLayout->addWidget(nameLineEdit);

    pushButtonLayout = new QHBoxLayout();
    pushButtonLayout->addWidget(getCameraSettingsButton);
    pushButtonLayout->addWidget(streamCameraSettingsButton);
    pushButtonLayout->addWidget(stopStreamCameraSettingsButton);

    saveImagesButtonLayout = new QHBoxLayout();
    saveImagesButtonLayout->addWidget(StartSavingImagesButton);
    saveImagesButtonLayout->addWidget(StopSavingImagesButton);
    saveImagesButtonLayout->addWidget(StartSavingImagesWithSettingsCheckBox);

    overallLayout = new QVBoxLayout();                 //!< Overall widget layout.
    overallLayout->addLayout(shiftGainLayout);
    overallLayout->addLayout(shiftExposureLayout);
    overallLayout->addLayout(shiftBlueChromaLayout);
    overallLayout->addLayout(shiftRedChromaLayout);
    overallLayout->addLayout(shiftBrightnessLayout);
    overallLayout->addLayout(shiftSaturationLayout);
    overallLayout->addLayout(shiftContrastLayout);
    overallLayout->addLayout(shiftHueLayout);
    overallLayout->addLayout(robotNameInputLayout);
    overallLayout->addLayout(pushButtonLayout);
    overallLayout->addLayout(saveImagesButtonLayout);

    setLayout(overallLayout);
}
void cameraSettingsWidget::createConnections()                    //!< Connect all of the child widgets.
{
    // Setup Shift Gain signals
    connect(shiftGainSlider,SIGNAL(valueChanged(int)),shiftGainSpinBox,SLOT(setValue(int)));
    connect(shiftGainSpinBox,SIGNAL(valueChanged(int)),shiftGainSlider,SLOT(setValue(int)));
    connect(shiftGainSlider,SIGNAL(valueChanged(int)),this,SLOT(cameraSettingsChanged()));

    // Setup Shift Exposure signals
    connect(shiftExposureSlider,SIGNAL(valueChanged(int)),shiftExposureSpinBox,SLOT(setValue(int)));
    connect(shiftExposureSpinBox,SIGNAL(valueChanged(int)),shiftExposureSlider,SLOT(setValue(int)));
    connect(shiftExposureSlider,SIGNAL(valueChanged(int)),this,SLOT(cameraSettingsChanged()));

    // Setup Shift BlueChroma signals
    connect(shiftBlueChromaSlider,SIGNAL(valueChanged(int)),shiftBlueChromaSpinBox,SLOT(setValue(int)));
    connect(shiftBlueChromaSpinBox,SIGNAL(valueChanged(int)),shiftBlueChromaSlider,SLOT(setValue(int)));
    connect(shiftBlueChromaSlider,SIGNAL(valueChanged(int)),this,SLOT(cameraSettingsChanged()));

    // Setup Shift RedChroma signals
    connect(shiftRedChromaSlider,SIGNAL(valueChanged(int)),shiftRedChromaSpinBox,SLOT(setValue(int)));
    connect(shiftRedChromaSpinBox,SIGNAL(valueChanged(int)),shiftRedChromaSlider,SLOT(setValue(int)));
    connect(shiftRedChromaSlider,SIGNAL(valueChanged(int)),this,SLOT(cameraSettingsChanged()));

    // Setup Shift Brightness signals
    connect(shiftBrightnessSlider,SIGNAL(valueChanged(int)),shiftBrightnessSpinBox,SLOT(setValue(int)));
    connect(shiftBrightnessSpinBox,SIGNAL(valueChanged(int)),shiftBrightnessSlider,SLOT(setValue(int)));
    connect(shiftBrightnessSlider,SIGNAL(valueChanged(int)),this,SLOT(cameraSettingsChanged()));

    // Setup Shift Saturation signals
    connect(shiftSaturationSlider,SIGNAL(valueChanged(int)),shiftSaturationSpinBox,SLOT(setValue(int)));
    connect(shiftSaturationSpinBox,SIGNAL(valueChanged(int)),shiftSaturationSlider,SLOT(setValue(int)));
    connect(shiftSaturationSlider,SIGNAL(valueChanged(int)),this,SLOT(cameraSettingsChanged()));

    // Setup Shift Contrast signals.
    connect(shiftContrastSlider,SIGNAL(valueChanged(int)),shiftContrastSpinBox,SLOT(setValue(int)));
    connect(shiftContrastSpinBox,SIGNAL(valueChanged(int)),shiftContrastSlider,SLOT(setValue(int)));
    connect(shiftContrastSlider,SIGNAL(valueChanged(int)),this,SLOT(cameraSettingsChanged()));

    // Setup Shift Hue signals
    connect(shiftHueSlider,SIGNAL(valueChanged(int)),shiftHueSpinBox,SLOT(setValue(int)));
    connect(shiftHueSpinBox,SIGNAL(valueChanged(int)),shiftHueSlider,SLOT(setValue(int)));
    connect(shiftHueSlider,SIGNAL(valueChanged(int)),this,SLOT(cameraSettingsChanged()));

    connect(nameLineEdit, SIGNAL(textChanged(QString)),this,SLOT(updateRobotName(QString)));
    connect(getCameraSettingsButton,SIGNAL(pressed()),this,SLOT(getCameraSetting()));
    connect(streamCameraSettingsButton,SIGNAL(pressed()),this,SLOT(streamCameraSetting()));
    connect(stopStreamCameraSettingsButton,SIGNAL(pressed()),this,SLOT(stopStreamCameraSetting()));

    connect(&readPacketTimer,SIGNAL(timeout()),this,SLOT(readPendingData()));
    connect(&timer,SIGNAL(timeout()),this,SLOT(sendSettingsToRobot()));
    connect(StartSavingImagesButton, SIGNAL(pressed()),this,SLOT(sendStartSavingImagesJob()));
    connect(StopSavingImagesButton, SIGNAL(pressed()),this,SLOT(sendStopSavingImagesJob()));
}


void cameraSettingsWidget::cameraSettingsChanged()
{

    settings->gain = shiftGainSlider->value();
    settings->exposure = shiftExposureSlider->value();
    settings->blueChroma = shiftBlueChromaSlider->value();
    settings->redChroma = shiftRedChromaSlider->value();
    settings->brightness = shiftBrightnessSlider->value();
    settings->saturation = shiftSaturationSlider->value();
    settings->contrast = shiftContrastSlider->value();
    settings->hue = shiftHueSlider->value();

}


void cameraSettingsWidget::updateRobotName(const QString name)
{
    getCameraSettingsButton->setEnabled(true);
    streamCameraSettingsButton->setEnabled(false);
    stopStreamCameraSettingsButton->setEnabled(false);
    if(stopStreamCameraSettingsButton->isEnabled() == true)
    {
        stopStreamCameraSetting();
    }
    robotName = name;
    //nuio->;
}
void cameraSettingsWidget::getCameraSetting()
{
    getCameraSettingsButton->setEnabled(true);
    streamCameraSettingsButton->setEnabled(true);
    stopStreamCameraSettingsButton->setEnabled(false);
    connectToRobot();
}
void cameraSettingsWidget::connectToRobot()
{
    /*tcpSocket->flush();
    netdata.clear();
    quint16 port = quint16(15438);
    if(tcpSocket->state() == QAbstractSocket::UnconnectedState)
    {
        tcpSocket->connectToHost(robotName,port,QIODevice::ReadWrite);

    }
    */
    sendDataToRobot();
}

void cameraSettingsWidget::sendDataToRobot()
{
    /*if(!(getCameraSettingsButton->isEnabled()) &&
       streamCameraSettingsButton->isEnabled() &&
       !(stopStreamCameraSettingsButton->isEnabled()))
    {
        const char* data = "1";
        if (tcpSocket->write(data) == -1)
        {
            qDebug() <<"Failed";
        }


        datasize = 0;
    }*/

    //USING THE JOB INTERFACE: (SENDS -1s to robot)
    CameraSettings tempSettings;
    tempSettings.gain = -1;
    tempSettings.exposure = -1;
    tempSettings.contrast = -1;

    static ChangeCameraSettingsJob* camerajob = new ChangeCameraSettingsJob(tempSettings);


    m_job_list->addCameraJob(camerajob);
    m_job_list->summaryTo(debug);

    (*nuio) << m_job_list;

    m_job_list->clear();

    readPacketTimer.start();

}

void cameraSettingsWidget::disconnectFromRobot()
{

   /* if(tcpSocket->isOpen())
    {
        //qDebug() << "disconnecting the Socket";
        tcpSocket->disconnectFromHost();
    }
    */

}
void cameraSettingsWidget::sendSettingsToRobot()
{


    //qDebug() << "Settings: " << settings->gain << ","<<settings->exposure;

    ChangeCameraSettingsJob* camerajob = new ChangeCameraSettingsJob(*settings);

    //qDebug() << "Settings: " << camerajob->getSettings().gain << ","<<camerajob->getSettings().exposure;

    m_job_list->addCameraJob(camerajob);
    m_job_list->summaryTo(debug);

    (*nuio) << m_job_list;

    m_job_list->clear();

}

void cameraSettingsWidget::readPendingData()
{
    /*if(netdata.isEmpty())
    {
        CameraSettings tempSettings;
        netdata.append(tcpSocket->readAll());
        qDebug() << "Got Data:" << netdata.size();
        std::stringstream buffer;
        buffer.write(reinterpret_cast<char*>(netdata.data()),netdata.size());
        buffer >> tempSettings;
        //Settings Display Values
        shiftExposureSlider->setValue(tempSettings.gain);
        shiftGainSlider->setValue(tempSettings.exposure);
        shiftBlueChromaSlider->setValue(tempSettings.blueChroma);
        shiftRedChromaSlider->setValue(tempSettings.redChroma);
        shiftBrightnessSlider->setValue(tempSettings.brightness);
        shiftSaturationSlider->setValue(tempSettings.saturation);
        shiftContrastSlider->setValue(tempSettings.contrast);
        shiftHueSlider->setValue(tempSettings.hue);
        settings->autoExposure = tempSettings.autoExposure;
        settings->autoGain = tempSettings.autoGain;
        settings->autoWhiteBalance = tempSettings.autoWhiteBalance;

    }*/

     //(*nuio) >> m_job_list;

        static list<Job*>::iterator it;     // the iterator over the motion jobs
        for (it = nuio->m_jobs->camera_begin(); it !=nuio->m_jobs->camera_end(); ++it)
        {
           // debug  << "Vision::Process - Processing Job" << endl;
            if ((*it)->getID() == Job::CAMERA_CHANGE_SETTINGS)
            {   // process a walk speed job
                //CameraSettings settings;
                static ChangeCameraSettingsJob* job;
                job = (ChangeCameraSettingsJob*) (*it);

                CameraSettings tempsettings = job->getSettings();
                if(tempsettings.exposure > 0 && tempsettings.exposure != settings->exposure)
                {
                    stopStreamCameraSetting();
                    //*settings = tempsettings;
                    debug << "Job Processed: " << endl;
                    shiftExposureSlider->setValue(tempsettings.exposure);
                    shiftGainSlider->setValue(tempsettings.gain);
                    shiftBlueChromaSlider->setValue(tempsettings.blueChroma);
                    shiftRedChromaSlider->setValue(tempsettings.redChroma);
                    shiftBrightnessSlider->setValue(tempsettings.brightness);
                    shiftSaturationSlider->setValue(tempsettings.saturation);
                    shiftContrastSlider->setValue(tempsettings.contrast);
                    shiftHueSlider->setValue(tempsettings.hue);
                }
            }
        }
        nuio->m_jobs->clear();

}

void cameraSettingsWidget::streamCameraSetting()
{
    getCameraSettingsButton->setEnabled(false);
    streamCameraSettingsButton->setEnabled(false);
    stopStreamCameraSettingsButton->setEnabled(true);
    dostream = true;
    readPacketTimer.stop();
    timer.start();
}
void cameraSettingsWidget::stopStreamCameraSetting()
{
    getCameraSettingsButton->setEnabled(true);
    streamCameraSettingsButton->setEnabled(true);
    stopStreamCameraSettingsButton->setEnabled(false);
    timer.stop();
    dostream = false;
    readPacketTimer.stop();

}


void cameraSettingsWidget::sendStartSavingImagesJob()
{
    SaveImagesJob* saveimagesjob = new SaveImagesJob(true, StartSavingImagesWithSettingsCheckBox->isChecked());
    m_job_list->addVisionJob(saveimagesjob);
    m_job_list->summaryTo(debug);

    (*nuio) << m_job_list;

    m_job_list->clear();

}

void cameraSettingsWidget::sendStopSavingImagesJob()
{
    SaveImagesJob* saveimagesjob = new SaveImagesJob(false);
    m_job_list->addVisionJob(saveimagesjob);
    m_job_list->summaryTo(debug);

    (*nuio) << m_job_list;

    m_job_list->clear();

}
