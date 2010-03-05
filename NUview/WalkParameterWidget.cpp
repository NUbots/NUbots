#include "WalkParameterWidget.h"

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
//#include <QStringlist>
#include <QPixmap>
#include <QPushButton>
#include <QSignalMapper>
#include <QPainter>
#include <QToolButton>
#include <QColorDialog>

#include <typeinfo>
#include "GLDisplay.h"

#include "../Motion/NUWalk.h"
#include "../Behaviour/Jobs.h"
#include "../NUPlatform/NUIO.h"
#include "../NUPlatform/NUSystem.h"
#include "../NUPlatform/Platforms/NAO/NAOSystem.h"
#include "debug.h"

WalkParameterWidget::WalkParameterWidget(QMdiArea* parentMdiWidget, QWidget *parent): QWidget(parent)
{
    setObjectName(tr("Walk Parameter(s)"));
    setWindowTitle(tr("Walk Parameter(s)"));

    createWidgets();
    createLayout();
    createConnections();
    this->setEnabled(true);
    disableWriting = false;
    
    debug.open("debug.log");
    errorlog.open("error.log");
    
    m_nusystem = new NAOSystem();
    nusystem = m_nusystem;
    m_job_list = new JobList();
    m_io = new NUIO(0);
    m_walk_parameters = new WalkParameters();
    
    ifstream testparafile("jupptestparameters.wp");
    if (testparafile.is_open())
        testparafile >> *m_walk_parameters;
}

void WalkParameterWidget::createWidgets()
{    
    // Shift Amplitude
    shiftAmplitudeLabel = new QLabel("Amplitude");
    shiftAmplitudeSlider = new QSlider(Qt::Horizontal);
    shiftAmplitudeSlider->setMinimum(1);
    shiftAmplitudeSlider->setMaximum(100);

    shiftAmplitudeSpinBox = new QSpinBox();
    shiftAmplitudeSpinBox->setMinimum(shiftAmplitudeSlider->minimum());
    shiftAmplitudeSpinBox->setMaximum(shiftAmplitudeSlider->maximum());
    
    // Shift Frequency
    shiftFrequencyLabel = new QLabel("Frequency");
    shiftFrequencySlider = new QSlider(Qt::Horizontal);
    shiftFrequencySlider->setMinimum(1);
    shiftFrequencySlider->setMaximum(100);
    
    shiftFrequencySpinBox = new QSpinBox();
    shiftFrequencySpinBox->setMinimum(shiftFrequencySlider->minimum());
    shiftFrequencySpinBox->setMaximum(shiftFrequencySlider->maximum());
    
    // Phase Offset
    phaseOffsetLabel = new QLabel("Phase");
    phaseOffsetSlider = new QSlider(Qt::Horizontal);
    phaseOffsetSlider->setMinimum(-100);
    phaseOffsetSlider->setMaximum(100);
    
    phaseOffsetSpinBox = new QSpinBox();
    phaseOffsetSpinBox->setMinimum(phaseOffsetSlider->minimum());
    phaseOffsetSpinBox->setMaximum(phaseOffsetSlider->maximum());
    
    // Phase Reset
    phaseResetLabel = new QLabel("Phase Reset");
    phaseResetSlider = new QSlider(Qt::Horizontal);
    phaseResetSlider->setMinimum(-100);
    phaseResetSlider->setMaximum(100);
    
    phaseResetSpinBox = new QSpinBox();
    phaseResetSpinBox->setMinimum(phaseResetSlider->minimum());
    phaseResetSpinBox->setMaximum(phaseResetSlider->maximum());
    
    // Step Size
    stepSizeLabel = new QLabel("Step Size");
    stepSizeSlider = new QSlider(Qt::Horizontal);
    stepSizeSlider->setMinimum(1);
    stepSizeSlider->setMaximum(400);
    
    stepSizeSpinBox = new QSpinBox();
    stepSizeSpinBox->setMinimum(stepSizeSlider->minimum());
    stepSizeSpinBox->setMaximum(stepSizeSlider->maximum());
}

void WalkParameterWidget::createLayout()
{
    // Shift Amplitude 
    shiftAmplitudeLayout = new QHBoxLayout();
    shiftAmplitudeLayout->addWidget(shiftAmplitudeLabel);
    shiftAmplitudeLayout->addWidget(shiftAmplitudeSlider);
    shiftAmplitudeLayout->addWidget(shiftAmplitudeSpinBox);
    
    // Shift Frequency 
    shiftFrequencyLayout = new QHBoxLayout();
    shiftFrequencyLayout->addWidget(shiftFrequencyLabel);
    shiftFrequencyLayout->addWidget(shiftFrequencySlider);
    shiftFrequencyLayout->addWidget(shiftFrequencySpinBox);
    
    // Phase Offset 
    phaseOffsetLayout = new QHBoxLayout();
    phaseOffsetLayout->addWidget(phaseOffsetLabel);
    phaseOffsetLayout->addWidget(phaseOffsetSlider);
    phaseOffsetLayout->addWidget(phaseOffsetSpinBox);
    
    // Phase Reset 
    phaseResetLayout = new QHBoxLayout();
    phaseResetLayout->addWidget(phaseResetLabel);
    phaseResetLayout->addWidget(phaseResetSlider);
    phaseResetLayout->addWidget(phaseResetSpinBox);
    
    // Step Size 
    stepSizeLayout = new QHBoxLayout();
    stepSizeLayout->addWidget(stepSizeLabel);
    stepSizeLayout->addWidget(stepSizeSlider);
    stepSizeLayout->addWidget(stepSizeSpinBox);

    // Setup overall layout
    overallLayout = new QVBoxLayout();
    overallLayout->addLayout(shiftAmplitudeLayout);
    overallLayout->addLayout(shiftFrequencyLayout);
    overallLayout->addLayout(phaseOffsetLayout);
    overallLayout->addLayout(phaseResetLayout);
    overallLayout->addLayout(stepSizeLayout);
    setLayout(overallLayout);
}

void WalkParameterWidget::createConnections()
{
    // Setup Shift Amplitude signals
    connect(shiftAmplitudeSlider,SIGNAL(valueChanged(int)),shiftAmplitudeSpinBox,SLOT(setValue(int)));
    connect(shiftAmplitudeSpinBox,SIGNAL(valueChanged(int)),shiftAmplitudeSlider,SLOT(setValue(int)));
    connect(shiftAmplitudeSlider,SIGNAL(valueChanged(int)),this,SLOT(walkParameterChanged()));
    
    // Setup Shift Frequency signals
    connect(shiftFrequencySlider,SIGNAL(valueChanged(int)),shiftFrequencySpinBox,SLOT(setValue(int)));
    connect(shiftFrequencySpinBox,SIGNAL(valueChanged(int)),shiftFrequencySlider,SLOT(setValue(int)));
    connect(shiftFrequencySlider,SIGNAL(valueChanged(int)),this,SLOT(walkParameterChanged()));
    
    // Setup Phase Offset signals
    connect(phaseOffsetSlider,SIGNAL(valueChanged(int)),phaseOffsetSpinBox,SLOT(setValue(int)));
    connect(phaseOffsetSpinBox,SIGNAL(valueChanged(int)),phaseOffsetSlider,SLOT(setValue(int)));
    connect(phaseOffsetSlider,SIGNAL(valueChanged(int)),this,SLOT(walkParameterChanged()));
    
    // Setup Phase Reset signals
    connect(phaseResetSlider,SIGNAL(valueChanged(int)),phaseResetSpinBox,SLOT(setValue(int)));
    connect(phaseResetSpinBox,SIGNAL(valueChanged(int)),phaseResetSlider,SLOT(setValue(int)));
    connect(phaseResetSlider,SIGNAL(valueChanged(int)),this,SLOT(walkParameterChanged()));
    
    // Setup Shift Amplitude signals
    connect(stepSizeSlider,SIGNAL(valueChanged(int)),stepSizeSpinBox,SLOT(setValue(int)));
    connect(stepSizeSpinBox,SIGNAL(valueChanged(int)),stepSizeSlider,SLOT(setValue(int)));
    connect(stepSizeSlider,SIGNAL(valueChanged(int)),this,SLOT(walkParameterChanged()));
}

WalkParameterWidget::~WalkParameterWidget()
{
    // Shift Amplitude
    delete shiftAmplitudeLabel;      
    delete shiftAmplitudeSlider;     
    delete shiftAmplitudeSpinBox;    
    delete shiftAmplitudeLayout;
    
    // Shift Frequency
    delete shiftFrequencyLabel;      
    delete shiftFrequencySlider;    
    delete shiftFrequencySpinBox;   
    delete shiftFrequencyLayout; 
    
    // Phase Offset
    delete phaseOffsetLabel;      
    delete phaseOffsetSlider;    
    delete phaseOffsetSpinBox;   
    delete phaseOffsetLayout; 
    
    // Phase Offset
    delete phaseResetLabel;      
    delete phaseResetSlider;    
    delete phaseResetSpinBox;   
    delete phaseResetLayout; 
    
    // Step Size
    delete stepSizeLabel;           
    delete stepSizeSlider;          
    delete stepSizeSpinBox;    
    delete stepSizeLayout; 
    
    delete overallLayout;
}

void WalkParameterWidget::walkParameterChanged()
{
    static WalkParametersJob* parametersjob = new WalkParametersJob(*m_walk_parameters);
    static vector<float> speed(3,0);
    static WalkJob* walkjob = new WalkJob(speed);
    static vector< vector<WalkParameters::Parameter> > params;

    m_walk_parameters->getParameters(params);
    
    // params[0] is shift frequency
    // params[2] is shift amplitude

    shiftAmplitudeSlider->value();

    params[0][0].Value = shiftFrequencySlider->value()/50.0;
    params[0][1].Value = phaseOffsetSlider->value()/100.0;
    params[0][2].Value = shiftAmplitudeSlider->value()/200.0;
    params[0][13].Value = phaseResetSlider->value()/100.0;
    speed[0] = stepSizeSlider->value()/10.0;

    m_walk_parameters->setParameters(params);
    
    parametersjob->setWalkParameters(*m_walk_parameters);
    walkjob->setSpeed(speed);
    m_job_list->addMotionJob(walkjob);
    m_job_list->addMotionJob(parametersjob);
    m_job_list->summaryTo(debug);

    (*m_io) << m_job_list;
    
    m_job_list->removeMotionJob(parametersjob);
    m_job_list->removeMotionJob(walkjob);
}


