#ifndef OFFLINELOCBATCH_H
#define OFFLINELOCBATCH_H

#include <QObject>
#include <QStringList>
#include <QThread>
#include <vector>
#include <iterator>
class OfflineLocalisation;
class LogFileReader;
class LocalisationSettings;

class LogLoaderThread : public QThread
{
    Q_OBJECT
public:
    void run();
public slots:
    void LoadLog(LogFileReader* reader, const QString& log_path);
private:
    LogFileReader* m_reader;
    QString m_log_path;
};

class ReportWriterThread : public QThread
{
    Q_OBJECT
public:
    void run();
public slots:
    void WriteReport(OfflineLocalisation* offline_loc, const QString& report_path);
private:
    OfflineLocalisation* m_offline_loc;
    QString m_report_path;
};

class OfflineLocBatch : public QObject
{
    Q_OBJECT
public:
    explicit OfflineLocBatch(QObject *parent = 0);
    ~OfflineLocBatch();
    QString statusMessage();
    bool isRunning(){return m_running;}
    unsigned int totalExperiments(){return m_file_list.size() * m_simulation_settings.size();}
    unsigned int currentExperimentNumber()
    {
        std::vector<LocalisationSettings*>::iterator setting = m_current_sim_settings;
        QStringList::Iterator file = m_current_file;
        const unsigned int experiments_per_file = m_simulation_settings.size();
        const unsigned int files_complete = std::distance(m_file_list.begin(), file);
        const unsigned int experiments_complete = std::distance(m_simulation_settings.begin(), setting);
        return files_complete*experiments_per_file + experiments_complete + 1;
    }
signals:
    void ProcessingComplete();
    void FileProgressChanged(const QString& current_file);
    void StatusChanged(const QString& status_message);
    void ProgressChanged(int, int);
    
public slots:
    void ProcessFiles(const QStringList& source_files, const QString& report_path, const QString& batch_type);
    void writeReport();
    void StopProcessing();

protected slots:
    void ExperimentComplete();
    void LogLoadComplete();

protected:
    QString ReportName(const LocalisationSettings& settings, const QString& log_name);
    std::vector<LocalisationSettings*> GenerateBranchMergeBatchSettings();
    std::vector<LocalisationSettings*> GenerateFilterExperimentBatchSettings();

    QString FindCommonPath(const QStringList& paths);

    void LoadLog(const QString& path);
    OfflineLocalisation* m_offline_loc;
    LogFileReader* m_log_reader;
    QString m_current_batch_type;
    QString m_common_source_path;
    QString m_result_path;
    QStringList m_file_list;
    QStringList::Iterator m_current_file;
    std::vector<LocalisationSettings*> m_simulation_settings;
    std::vector<LocalisationSettings*>::iterator m_current_sim_settings;
    bool m_running;
    LogLoaderThread* m_log_loader;
    ReportWriterThread* m_report_writer;

};

#endif // OFFLINELOCBATCH_H
