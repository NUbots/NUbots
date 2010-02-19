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
#include <QStringlist>
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
}

void WalkParameterWidget::createWidgets()
{    
    // Shift Amplitude
    shiftAmplitudeLabel = new QLabel("Amplitude");
    shiftAmplitudeSlider = new QSlider(Qt::Horizontal);
    shiftAmplitudeSlider->setMinimum(0);
    shiftAmplitudeSlider->setMaximum(100);

    shiftAmplitudeSpinBox = new QSpinBox();
    shiftAmplitudeSpinBox->setMinimum(shiftAmplitudeSlider->minimum());
    shiftAmplitudeSpinBox->setMaximum(shiftAmplitudeSlider->maximum());
    
    // Shift Frequency
    shiftFrequencyLabel = new QLabel("Frequency");
    shiftFrequencySlider = new QSlider(Qt::Horizontal);
    shiftFrequencySlider->setMinimum(0);
    shiftFrequencySlider->setMaximum(100);
    
    shiftFrequencySpinBox = new QSpinBox();
    shiftFrequencySpinBox->setMinimum(shiftFrequencySlider->minimum());
    shiftFrequencySpinBox->setMaximum(shiftFrequencySlider->maximum());
    
    // Step Size
    stepSizeLabel = new QLabel("Step Size");
    stepSizeSlider = new QSlider(Qt::Horizontal);
    stepSizeSlider->setMinimum(0);
    stepSizeSlider->setMaximum(100);
    
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
    
    // Step Size 
    stepSizeLayout = new QHBoxLayout();
    stepSizeLayout->addWidget(stepSizeLabel);
    stepSizeLayout->addWidget(stepSizeSlider);
    stepSizeLayout->addWidget(stepSizeSpinBox);

    // Setup overall layout
    overallLayout = new QVBoxLayout();
    overallLayout->addLayout(shiftAmplitudeLayout);
    overallLayout->addLayout(shiftFrequencyLayout);
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
    
    // Step Size
    delete stepSizeLabel;           
    delete stepSizeSlider;          
    delete stepSizeSpinBox;    
    delete stepSizeLayout; 
    
    delete overallLayout;
}

void WalkParameterWidget::walkParameterChanged()
{
    shiftAmplitudeSlider->value();
    shiftFrequencySlider->value();
    stepSizeSlider->value();
    
    m_job_list->summaryTo(debug);
    (*m_io) << m_job_list;
}


