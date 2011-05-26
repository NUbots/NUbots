#include "offlinelocalisationdialog.h"
#include "OfflineLocalisation.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextBrowser>
#include <QPushButton>
#include <QFileDialog>
#include <QDir>
#include <QFileInfo>
#include <QLabel>
#include <QProgressDialog>
#include "FileAccess/SplitStreamFileFormatReader.h"

OfflineLocalisationDialog::OfflineLocalisationDialog(QWidget *parent) :
    QDialog(parent)
{
    m_offline_loc = new OfflineLocalisation();

    QVBoxLayout *buttonsLayout = new QVBoxLayout();

    QPushButton *openFileButton = new QPushButton("&Open Log...");
    connect(openFileButton,SIGNAL(pressed()), this, SLOT(OpenLogFiles()));
    buttonsLayout->addWidget(openFileButton);

    QPushButton *runSimulationButton = new QPushButton("&Run simulation");
    connect(runSimulationButton,SIGNAL(pressed()), this, SLOT(BeginSimulation()));
    buttonsLayout->addWidget(runSimulationButton);

    QVBoxLayout *displayLayout = new QVBoxLayout();

    QLabel *fileLabel = new QLabel("Log files");
    displayLayout->addWidget(fileLabel);

    fileListDisplay = new QTextBrowser(this);
    fileListDisplay->setWordWrapMode(QTextOption::NoWrap);

    displayLayout->addWidget(fileListDisplay);


    QHBoxLayout *overallLayout = new QHBoxLayout();

    overallLayout->addLayout(buttonsLayout);
    overallLayout->addLayout(displayLayout,1);
    m_progressBar = NULL;
    setLayout(overallLayout);
}

OfflineLocalisationDialog::~OfflineLocalisationDialog()
{
    if(m_offline_loc) delete m_offline_loc;
}

void OfflineLocalisationDialog::OpenLogFiles()
{
    QString filename = QFileDialog::getOpenFileName(this,
                            tr("Open Replay File"), ".",
                            tr("All NUbot Image Files(*.nul;*.nif;*.nurf;*.strm);;NUbot Log Files (*.nul);;NUbot Image Files (*.nif);;NUbot Replay Files (*.nurf);;Stream File(*.strm);;All Files(*.*)"));

    if (!filename.isEmpty()){
        m_offline_loc->OpenLogs(filename.toStdString());
    }

    QFileInfo fileInfo(filename);
    QDir openDir;
    openDir.setPath(fileInfo.path());
    std::vector<QFileInfo> logs = SplitStreamFileFormatReader::FindValidFiles(openDir);
    QString displayString;

    std::vector<QFileInfo>::const_iterator fileIt;
    for (fileIt = logs.begin(); fileIt != logs.end(); ++fileIt)
    {
        displayString += (*fileIt).filePath() + '\n';
    }
    fileListDisplay->setText(displayString);
}

void OfflineLocalisationDialog::BeginSimulation()
{
    if(m_offline_loc->isRunning()) return;
    m_progressBar = new QProgressDialog("Runing localisation...","Cancel",0, m_offline_loc->NumberOfLogFrames(),this);
    m_progressBar->setWindowModality(Qt::WindowModal);
    m_progressBar->setValue(0);
    m_progressBar->setMinimumDuration(100);
    connect(m_offline_loc, SIGNAL(updateProgress(int,int)), this, SLOT(DiplayProgress(int,int)));
    connect(m_offline_loc, SIGNAL(finished()), this, SLOT(CompleteSimulation()));
    connect(m_progressBar, SIGNAL(canceled()), this, SLOT(CancelProgress()));

    //m_offline_loc->Run();
    m_offline_loc->start();


}

void OfflineLocalisationDialog::CompleteSimulation()
{
    disconnect(m_offline_loc, SIGNAL(finished()), this, SLOT(CompleteSimulation()));
    disconnect(m_offline_loc, SIGNAL(updateProgress(int,int)), this, SLOT(DiplayProgress(int,int)));
    delete m_progressBar;
    m_progressBar = NULL;
}

void OfflineLocalisationDialog::DiplayProgress(int frame, int total)
{
    //qDebug("Processing frame %d of %d", frame, total);
    if(m_progressBar)
    {
        QString labelText = QString("Running Localisation... Frame %1 of %2").arg(frame).arg(total);
        m_progressBar->setLabelText(labelText);
        m_progressBar->setValue(frame);
    }
}

void OfflineLocalisationDialog::CancelProgress()
{
    if(m_offline_loc->isRunning()) m_offline_loc->stop();
}

void OfflineLocalisationDialog::SetFrame(int frameNumber, int total)
{
    if(m_offline_loc->hasSimData())
    {
        const Localisation* temp = m_offline_loc->GetFrame(frameNumber);
        emit LocalisationChanged(temp);
    }
}
