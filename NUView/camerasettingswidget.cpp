#include "camerasettingswidget.h"
// Qt Includes
#include <QMdiArea>
#include <QComboBox>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
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


#include "NUViewIO/NUViewIO.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/Jobs/Jobs.h"
#include "Infrastructure/Jobs/CameraJobs/ChangeCameraSettingsJob.h"
#include "Infrastructure/Jobs/VisionJobs/SaveImagesJob.h"
#include "NUPlatform/NUCamera/CameraSettings.h"
#include "debug.h"


cameraSettingsWidget::cameraSettingsWidget(QMdiArea* parentMdiWidget, QWidget *parent): QWidget(parent)
{
    setObjectName(tr("Camera"));
    setWindowTitle(tr("Camera"));
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

    TopCameraSelected = new QCheckBox("Top Camera");
    TopCameraSelected->setChecked(false);
    BottomCameraSelected = new QCheckBox("Bottom Camera");
    BottomCameraSelected->setChecked(true);
    AutoGainSelected = new QCheckBox("Auto Gain");
    AutoWhiteBalanceSelected = new QCheckBox("Auto White Balance");
    AutoExposureSelected = new QCheckBox("Auto Exposure");


}
void cameraSettingsWidget::createLayout()   //!< Layout all of the child widgets.
{
    SliderLayout = new QGridLayout;
    // Shift Gain
    //shiftGainLayout = new QHBoxLayout();                      //!< Layout for shift Gain
    SliderLayout->addWidget(shiftGainLabel,1,1);               //!< Label for shift Gain
    SliderLayout->addWidget(shiftGainSlider,1,2);              //!< Slider for Gain selection
    SliderLayout->addWidget(shiftGainSpinBox,1,3);             //!< SpinBox for Gain selection

    // Shift Exposure
    //shiftExposureLayout = new QHBoxLayout();                        //!< Layout for shift Exposure
    SliderLayout->addWidget(shiftExposureLabel,2,1);             //!< Label for shift Exposure
    SliderLayout->addWidget(shiftExposureSlider,2,2);            //!< Slider for Exposure selection
    SliderLayout->addWidget(shiftExposureSpinBox,2,3);           //!< SpinBox for Exposure selection


    // Shift BlueChroma
    //shiftBlueChromaLayout = new QHBoxLayout();                          //!< Layout for shift BlueChroma
    SliderLayout->addWidget(shiftBlueChromaLabel,3,1);             //!< Label for shift BlueChroma
    SliderLayout->addWidget(shiftBlueChromaSlider,3,2);            //!< Slider for Exposure BlueChroma
    SliderLayout->addWidget(shiftBlueChromaSpinBox,3,3);           //!< SpinBox for Exposure BlueChroma

    // Shift RedChroma
    //shiftRedChromaLayout = new QHBoxLayout();                          //!< Layout for shift RedChroma
    SliderLayout->addWidget(shiftRedChromaLabel,4,1);             //!< Label for shift RedChroma
    SliderLayout->addWidget(shiftRedChromaSlider,4,2);            //!< Slider for RedChroma selection
    SliderLayout->addWidget(shiftRedChromaSpinBox,4,3);           //!< SpinBox for RedChroma selection

    // Shift Brightness
    //shiftBrightnessLayout = new QHBoxLayout();                          //!< Layout for shift Brightness
    SliderLayout->addWidget(shiftBrightnessLabel,5,1);             //!< Label for shift Brightness
    SliderLayout->addWidget(shiftBrightnessSlider,5,2);            //!< Slider for Brightness selection
    SliderLayout->addWidget(shiftBrightnessSpinBox,5,3);           //!< SpinBox for Brightness selection

    // Shift Saturation
    //shiftSaturationLayout = new QHBoxLayout();                          //!< Layout for shift Saturation
    SliderLayout->addWidget(shiftSaturationLabel,6,1);             //!< Label for shift Saturation
    SliderLayout->addWidget(shiftSaturationSlider,6,2);            //!< Slider for Saturation selection
    SliderLayout->addWidget(shiftSaturationSpinBox,6,3);           //!< SpinBox for Saturation selection

    // Shift Contrast
    //shiftContrastLayout = new QHBoxLayout();                          //!< Layout for shift Contrast
    SliderLayout->addWidget(shiftContrastLabel,7,1);             //!< Label for shift Contrast
    SliderLayout->addWidget(shiftContrastSlider,7,2);            //!< Slider for Contrast selection
    SliderLayout->addWidget(shiftContrastSpinBox,7,3);           //!< SpinBox for Contrast selection

    // Shift Hue
    //shiftHueLayout = new QHBoxLayout();                   //!< Layout for shift Hue
    SliderLayout->addWidget(shiftHueLabel,8,1);             //!< Label for shift Hue
    SliderLayout->addWidget(shiftHueSlider,8,2);            //!< Slider for Hue selection
    SliderLayout->addWidget(shiftHueSpinBox,8,3);           //!< SpinBox for Hue selection


    CameraSettingsGroupBox = new QGroupBox(tr("Camera Settings Sliders"));
    CameraSettingsGroupBox->setLayout(SliderLayout);

    //Auto Settings Layout:
    AutoTickBoxLayout = new QGridLayout;
    AutoTickBoxLayout->addWidget(AutoGainSelected,1,1);
    AutoTickBoxLayout->addWidget(AutoWhiteBalanceSelected,2,1);
    AutoTickBoxLayout->addWidget(AutoExposureSelected,3,1);
    AutoCameraSettingsGroupBox = new QGroupBox(tr("Automatic Settings"));
    AutoCameraSettingsGroupBox->setLayout(AutoTickBoxLayout);

    //Select Camera:
    SelectCameraLayout = new QGridLayout;
    SelectCameraLayout->addWidget(TopCameraSelected,1,1);
    SelectCameraLayout->addWidget(BottomCameraSelected,2,1);
    SelectCameraGroupBox = new QGroupBox(tr("Select Camera"));
    SelectCameraGroupBox->setLayout(SelectCameraLayout);

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
    //overallLayout->addLayout(shiftGainLayout);
    //overallLayout->addLayout(shiftExposureLayout);
    //overallLayout->addLayout(shiftBlueChromaLayout);
    //overallLayout->addLayout(shiftRedChromaLayout);
    //overallLayout->addLayout(shiftBrightnessLayout);
    //overallLayout->addLayout(shiftSaturationLayout);
    //overallLayout->addLayout(shiftContrastLayout);
    //overallLayout->addLayout(shiftHueLayout);
    overallLayout->addLayout(robotNameInputLayout);
    overallLayout->addWidget(CameraSettingsGroupBox);
    overallLayout->addWidget(AutoCameraSettingsGroupBox);
    overallLayout->addWidget(SelectCameraGroupBox);

    overallLayout->addWidget(getCameraSettingsButton);
    overallLayout->addWidget(streamCameraSettingsButton);
    overallLayout->addWidget(stopStreamCameraSettingsButton);
    overallLayout->addWidget(StartSavingImagesButton);
    overallLayout->addWidget(StopSavingImagesButton);
    overallLayout->addWidget(StartSavingImagesWithSettingsCheckBox);
    //overallLayout->addLayout(pushButtonLayout);
    //overallLayout->addLayout(saveImagesButtonLayout);

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

    // Setup Auto:
    //QCheckBox* TopCameraSelected;
    //QCheckBox* BottomCameraSelected;
    connect(AutoGainSelected,SIGNAL(clicked()),this,SLOT(cameraSettingsChanged()));
    connect(AutoWhiteBalanceSelected,SIGNAL(clicked()),this,SLOT(cameraSettingsChanged()));
    connect(AutoExposureSelected,SIGNAL(clicked()),this,SLOT(cameraSettingsChanged()));

    connect(TopCameraSelected,SIGNAL(clicked()),BottomCameraSelected,SLOT(toggle()));
    connect(BottomCameraSelected,SIGNAL(clicked()),TopCameraSelected,SLOT(toggle()));
    connect(TopCameraSelected,SIGNAL(clicked()),this,SLOT(cameraSettingsChanged()));
    connect(BottomCameraSelected,SIGNAL(clicked()),this,SLOT(cameraSettingsChanged()));


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

    settings->p_gain.set(shiftGainSlider->value());
    settings->p_exposure.set(shiftExposureSlider->value());
    settings->p_blueChroma.set(shiftBlueChromaSlider->value());
    settings->p_redChroma.set(shiftRedChromaSlider->value());
    settings->p_brightness.set(shiftBrightnessSlider->value());
    settings->p_saturation.set(shiftSaturationSlider->value());
    settings->p_contrast.set(shiftContrastSlider->value());
    settings->p_hue.set(shiftHueSlider->value());

    if(AutoExposureSelected->isChecked())
        settings->p_autoExposure.set(1);
    else
        settings->p_autoExposure.set(0);

    if(AutoGainSelected->isChecked())
        settings->p_autoGain.set(1);
    else
        settings->p_autoGain.set(0);

    if(AutoWhiteBalanceSelected->isChecked())
        settings->p_autoWhiteBalance.set(1);
    else
        settings->p_autoWhiteBalance.set(0);


    if(TopCameraSelected->isChecked())
        settings->activeCamera = (CameraSettings::TOP_CAMERA);
    else
        settings->activeCamera = (CameraSettings::BOTTOM_CAMERA);

    if(BottomCameraSelected->isChecked())
        settings->activeCamera = (CameraSettings::BOTTOM_CAMERA);
    else
        settings->activeCamera = (CameraSettings::TOP_CAMERA);

    settings->copyParams();

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
    std::string ipaddress = robotName.toStdString();
    nuio->setJobAddress(ipaddress);
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
    tempSettings.p_gain.set(0);
    tempSettings.p_exposure.set(0);
    tempSettings.p_contrast.set(0);
    tempSettings.copyParams();
    qDebug() << tempSettings.gain << tempSettings.exposure << tempSettings.contrast;
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


    qDebug() << "Camera Settings: Send " << settings->gain << ","<<settings->exposure;

    ChangeCameraSettingsJob* camerajob = new ChangeCameraSettingsJob(*settings);

    //qDebug() << "Settings: " << camerajob->getSettings().gain << ","<<camerajob->getSettings().exposure;

    m_job_list->addCameraJob(camerajob);
    m_job_list->summaryTo(debug);

    (*nuio) << m_job_list;

    m_job_list->clear();

    qDebug() << "Camera Settings: Sent " ;

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
        for (it = Blackboard->Jobs->camera_begin(); it !=Blackboard->Jobs->camera_end(); ++it)
        {
            qDebug()  << "CameraSettings - Processing Recieved Job" << endl;
            if ((*it)->getID() == Job::CAMERA_CHANGE_SETTINGS)
            {   // process a walk speed job
                //CameraSettings settings;
                static ChangeCameraSettingsJob* job;
                job = (ChangeCameraSettingsJob*) (*it);

                CameraSettings tempsettings = job->getSettings();
                if(tempsettings.exposure > 0)
                {
                    stopStreamCameraSetting();
                    //*settings = tempsettings;
                    debug << "Job Processed: " << endl;
                    shiftExposureSlider->setValue(round(tempsettings.p_exposure.get()));
                    shiftGainSlider->setValue(round(tempsettings.p_gain.get()));
                    shiftBlueChromaSlider->setValue(round(tempsettings.p_blueChroma.get()));
                    shiftRedChromaSlider->setValue(round(tempsettings.p_redChroma.get()));
                    shiftBrightnessSlider->setValue(round(tempsettings.p_brightness.get()));
                    shiftSaturationSlider->setValue(round(tempsettings.p_saturation.get()));
                    shiftContrastSlider->setValue(round(tempsettings.p_contrast.get()));
                    shiftHueSlider->setValue(round(tempsettings.p_hue.get()));
                    qDebug() << tempsettings.p_gain.get() << tempsettings.p_exposure.get() << tempsettings.p_contrast.get();

                    if(tempsettings.autoExposure == 1)
                       AutoExposureSelected->setChecked(true);
                    else
                       AutoExposureSelected->setChecked(false);

                    if(tempsettings.autoGain == 1)
                        AutoGainSelected->setChecked(true);
                    else
                        AutoGainSelected->setChecked(false);

                    if(tempsettings.autoWhiteBalance == 1)
                        AutoWhiteBalanceSelected->setChecked(true);
                    else
                        AutoWhiteBalanceSelected->setChecked(false);


                    if(tempsettings.activeCamera == CameraSettings::TOP_CAMERA)
                    {
                        TopCameraSelected->setChecked(true);
                        BottomCameraSelected->setChecked(false);
                    }
                    else
                    {
                        TopCameraSelected->setChecked(false);
                        BottomCameraSelected->setChecked(true);
                    }
                    qDebug()  << "CameraSettings - Processed" << endl;
                }
            }
        }
        Blackboard->Jobs->clear();
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
