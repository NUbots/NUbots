#include "KickWidget.h"

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

#include "../Behaviour/Jobs.h"
#include "../NUPlatform/NUIO.h"
#include "../NUPlatform/NUSystem.h"
#include "../NUPlatform/Platforms/NAO/NAOSystem.h"
#include "debug.h"

KickWidget::KickWidget(QMdiArea* parentMdiWidget, QWidget *parent): QWidget(parent)
{
    setObjectName(tr("Kick"));
    setWindowTitle(tr("Kick"));

    createWidgets();
    createLayout();
    createConnections();
    this->setEnabled(true);
    disableWriting = false;
    
    m_nusystem = new NAOSystem();
    nusystem = m_nusystem;
    m_job_list = new JobList();
    m_io = new NUIO(0, NULL);
}

void KickWidget::createWidgets()
{   
    // Kick Position
    positionLabel = new QLabel("Kick Position (x,y) mm");
    positionXSpinBox = new QSpinBox();
    positionXSpinBox->setMinimum(-100);
    positionXSpinBox->setMaximum(100);
    positionYSpinBox = new QSpinBox();
    positionYSpinBox->setMinimum(-100);
    positionYSpinBox->setMaximum(100);
    
    // Kick Target
    targetLabel = new QLabel("Kick Target (x,y) cm");
    targetXSpinBox = new QSpinBox();
    targetXSpinBox->setMinimum(-600);
    targetXSpinBox->setMaximum(600);
    targetYSpinBox = new QSpinBox();
    targetYSpinBox->setMinimum(-600);
    targetYSpinBox->setMaximum(600);
    
    // Kick Button
    kickButton= new QPushButton("Kick!");
}

void KickWidget::createLayout()
{
    // Kick Position 
    positionLayout = new QHBoxLayout();
    positionLayout->addWidget(positionLabel);
    positionLayout->addWidget(positionXSpinBox);
    positionLayout->addWidget(positionYSpinBox);
    
    // Kick Target 
    targetLayout = new QHBoxLayout();
    targetLayout->addWidget(targetLabel);
    targetLayout->addWidget(targetXSpinBox);
    targetLayout->addWidget(targetYSpinBox);
    
    // Kick Button
    kickButtonLayout = new QHBoxLayout();
    kickButtonLayout->addWidget(kickButton);

    // Setup overall layout
    overallLayout = new QVBoxLayout();
    overallLayout->addLayout(positionLayout);
    overallLayout->addLayout(targetLayout);
    overallLayout->addLayout(kickButtonLayout);
    setLayout(overallLayout);
}

void KickWidget::createConnections()
{
    // Setup Shift Amplitude signals
    connect(kickButton,SIGNAL(pressed()),this,SLOT(kickPressed()));
}

KickWidget::~KickWidget()
{
    // Kick Position
    delete positionLayout;                //!< Layout for kick position
    delete positionLabel;                      //!< Label for kick position
    delete positionXSpinBox;                 //!< SpinBox for kick position X
    delete positionYSpinBox;                 //!< SpinBox for kick position Y
    
    // Kick Target
    delete targetLayout;                  //!< Layout for target position
    delete targetLabel;                        //!< Label for target position
    delete targetXSpinBox;                   //!< SpinBox for target position X
    delete targetYSpinBox;                   //!< SpinBox for target position Y
    
    // Kick Button
    delete kickButtonLayout;              //!< Laout fo the kick button
    delete kickButton;                    //!< Button to trigger sending of kick job to robot
    
    delete m_nusystem;
    delete m_io;
    delete m_job_list; 
    
    delete overallLayout;
}

void KickWidget::kickPressed()
{
    static vector<float> kickposition(2,0);
    static vector<float> kicktarget(2,0);
    static KickJob* kickjob = new KickJob(0, kickposition, kicktarget);

    kickposition[0] = positionXSpinBox->value()/10.0;
    kickposition[1] = positionYSpinBox->value()/10.0;
    
    kicktarget[0] = targetXSpinBox->value();
    kicktarget[1] = targetYSpinBox->value();
    
    kickjob->setKick(0, kickposition, kicktarget);
    m_job_list->addMotionJob(kickjob);
    m_job_list->summaryTo(debug);

    (*m_io) << m_job_list;
    
    m_job_list->removeMotionJob(kickjob);
}


