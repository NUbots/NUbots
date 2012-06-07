#include "OfflineLocBatch.h"
#include "Localisation/LocalisationSettings.h"
#include "OfflineLocalisation.h"
#include "FileAccess/LogFileReader.h"
#include <QDebug>
#include <QDir>
#include <QFileInfo>

void LogLoaderThread::run()
{
    int frames = m_reader->openFile(m_log_path);
}

void LogLoaderThread::LoadLog(LogFileReader* reader, const QString& log_path)
{
    m_reader = reader;
    m_log_path = log_path;
    start();
}

void ReportWriterThread::run()
{
    if(!m_report_path.isEmpty() and m_offline_loc->hasSimData())
    {
        m_offline_loc->WriteXML(m_report_path.toStdString());
    }
    return;
}

void ReportWriterThread::WriteReport(OfflineLocalisation* offline_loc, const QString& report_path)
{
    m_offline_loc = offline_loc;
    m_report_path = report_path;
    start();
}

OfflineLocBatch::OfflineLocBatch(QObject *parent) :
    QObject(parent), m_running(false)
{
    GenerateSettings();
    m_log_reader = new LogFileReader(this);
    m_offline_loc = new OfflineLocalisation(m_log_reader, this);
    m_log_loader = new LogLoaderThread();
    m_report_writer = new ReportWriterThread();
    connect(m_log_loader, SIGNAL(finished()), this, SLOT(LogLoadComplete()), Qt::UniqueConnection);
    connect(m_report_writer, SIGNAL(finished()), this, SLOT(ExperimentComplete()), Qt::UniqueConnection);
}

OfflineLocBatch::~OfflineLocBatch()
{
    disconnect(m_log_loader, SIGNAL(finished()), this, SLOT(LogLoadComplete()));
    disconnect(m_report_writer, SIGNAL(finished()), this, SLOT(ExperimentComplete()));

    // Delete list of settings
    while(!m_simulation_settings.empty())
    {
        delete m_simulation_settings.back();    // delete object
        m_simulation_settings.pop_back();       // remove pointer
    }

    delete m_offline_loc;
    delete m_log_reader;
    delete m_log_loader;
    delete m_report_writer;
    m_file_list.clear();
}


void OfflineLocBatch::GenerateSettings()
{
    // Delete list of settings
    while(!m_simulation_settings.empty())
    {
        delete m_simulation_settings.back();    // delete object
        m_simulation_settings.pop_back();       // remove pointer
    }

    LocalisationSettings* loc = new LocalisationSettings();

    // make vector of branch methods.
    std::vector<LocalisationSettings::BranchMethod> branch_methods;
    branch_methods.push_back(LocalisationSettings::branch_exhaustive);
    branch_methods.push_back(LocalisationSettings::branch_selective);
    branch_methods.push_back(LocalisationSettings::branch_constraint);

    // make vector of pruning methods.
    std::vector<LocalisationSettings::PruneMethod> prune_methods;
    prune_methods.push_back(LocalisationSettings::prune_max_likelyhood);
    prune_methods.push_back(LocalisationSettings::prune_merge);
    prune_methods.push_back(LocalisationSettings::prune_nscan);
    prune_methods.push_back(LocalisationSettings::prune_viterbi);

    // make a setting for each combination.
    std::vector<LocalisationSettings::BranchMethod>::iterator branch_it = branch_methods.begin();
    for(;branch_it != branch_methods.end(); ++branch_it)
    {
        loc->setBranchMethod(*branch_it);
        std::vector<LocalisationSettings::PruneMethod>::iterator prune_it = prune_methods.begin();
        for (;prune_it != prune_methods.end(); ++prune_it)
        {
            loc->setPruneMethod(*prune_it);
            m_simulation_settings.push_back(new LocalisationSettings(*loc));
        }
    }

    // add probabalistic data association.
    loc->setBranchMethod(LocalisationSettings::branch_probDataAssoc);
    loc->setPruneMethod(LocalisationSettings::prune_unknown);
    m_simulation_settings.push_back(new LocalisationSettings(*loc));
    delete loc;
}


void OfflineLocBatch::ProcessFiles(const QStringList& files)
{
    m_running = true;
    m_file_list = files;
    m_current_file = m_file_list.begin();
    m_current_sim_settings = m_simulation_settings.begin();


    m_offline_loc->setSettings(*(*m_current_sim_settings));
    //connect(m_offline_loc, SIGNAL(finished()), this, SLOT(ExperimentComplete()), Qt::UniqueConnection);
    connect(m_offline_loc, SIGNAL(finished()), this, SLOT(writeReport()), Qt::UniqueConnection);
    connect(m_offline_loc, SIGNAL(updateProgress(int,int)), this, SIGNAL(ProgressChanged(int,int)), Qt::UniqueConnection);
    qDebug() << statusMessage();

    LoadLog(*m_current_file);
//    m_offline_loc->start();
    //emit StatusChanged(status());
}

void OfflineLocBatch::StopProcessing()
{
    if(m_log_loader->isRunning()) m_log_loader->terminate();
    disconnect(m_offline_loc, SIGNAL(updateProgress(int,int)), this, SIGNAL(ProgressChanged(int,int)));
    m_offline_loc->stop();
    disconnect(m_offline_loc, SIGNAL(finished()), this, SLOT(writeReport()));
    emit ProcessingComplete();
    qDebug("Processing cancelled.");
    return;
}

void OfflineLocBatch::writeReport()
{
    const QString message = "Saving report: %1";
    QString currentFile = *m_current_file;
    QFileInfo info(currentFile);

    QString log_name = ReportName(**m_current_sim_settings, info.baseName());
    emit ProgressChanged(0,0);
    emit StatusChanged(message.arg(log_name));
    QString log_path = info.absoluteDir().filePath(log_name);
    m_report_writer->WriteReport(m_offline_loc, log_path);
}

QString OfflineLocBatch::ReportName(const LocalisationSettings& settings, const QString& log_name)
{
    const QString extension = ".xml";
    // Report format: 'log name'_'Split method'_'merge method'
    QString report_name = log_name + '_' + QString(settings.branchMethodString().c_str()).replace(' ','_') + '_' + QString(settings.pruneMethodString().c_str()).replace(' ','_');
    return report_name.append(extension);
}

void OfflineLocBatch::ExperimentComplete()
{
    bool complete = false;
    bool new_file = false;
    ++m_current_sim_settings;
    if(m_current_sim_settings == m_simulation_settings.end())
    {
        m_current_sim_settings = m_simulation_settings.begin();
        ++m_current_file;
        if(m_current_file == m_file_list.end())
        {
           complete = true;
        }
        else
        {
            //LoadLog(*m_current_file);
            new_file = true;
        }
    }

    if(!complete)
    {
        qDebug("Running next experiment.");
        m_offline_loc->setSettings(*(*m_current_sim_settings));
        if(new_file) LoadLog(*m_current_file);
        else m_offline_loc->start();
        qDebug() << statusMessage();
    }
    else
    {
        m_running = false;
        qDebug("Batch Complete!");
        emit ProcessingComplete();
        disconnect(m_offline_loc, SIGNAL(finished()), this, SLOT(writeReport()));
        disconnect(m_offline_loc, SIGNAL(updateProgress(int,int)), this, SIGNAL(ProgressChanged(int,int)));

    }
}

void OfflineLocBatch::LoadLog(const QString& path)
{
    QString message = "Loading File: %1";
    emit ProgressChanged(0,0);
    emit StatusChanged(message.arg(path));

    QDir temp_dir(path);
    QString file_path = temp_dir.absoluteFilePath("none");
    //m_offline_loc->OpenLogs(file_path.toStdString());
    m_log_loader->LoadLog(m_log_reader, file_path);
    emit FileProgressChanged(path);
}

void OfflineLocBatch::LogLoadComplete()
{
    m_offline_loc->start();
}


QString OfflineLocBatch::statusMessage()
{
    QString status_message = "Experiment %1 of %2\nCurrent File: %3\n branch method: %4\nprune method: %5";
    return status_message.arg(currentExperimentNumber()).arg(totalExperiments()).arg(*m_current_file, QString((*m_current_sim_settings)->branchMethodString().c_str()), QString((*m_current_sim_settings)->pruneMethodString().c_str()));
}
