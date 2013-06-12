#ifndef OFFLINELOCALISATIONDIALOG_H
#define OFFLINELOCALISATIONDIALOG_H

#include <QDialog>
#include <QFileInfo>
#include <vector>
#include <QStringList>
#include <QThread>
#include "Localisation/LocalisationSettings.h"
class OfflineLocalisation;
class QTextBrowser;
class QProgressDialog;
class Localisation;
class SelfLocalisation;
class LogFileReader;
class OfflineLocBatch;
class QLineEdit;
class QLabel;
class QComboBox;

class OfflineLocalisationDialog : public QDialog
{
    Q_OBJECT
public:
    explicit OfflineLocalisationDialog(QWidget *parent = 0);
    explicit OfflineLocalisationDialog(LogFileReader* reader, QWidget *parent = 0);
    ~OfflineLocalisationDialog();

signals:
    void LocalisationChanged(const Localisation*);
    void SelfLocalisationChanged(const SelfLocalisation*);
    void LocalisationInfoChanged(const QString&);
    void SelfLocalisationInfoChanged(const QString&);
    void PostBatchJob(const QStringList&, const QString&, const QString&);
    void ProcessingStateChanged(bool isProcessing);

public slots:
    void OpenLogFiles();
    void BeginSimulation();
    void BeginBatch();
    void CompleteSimulation();
    void ValidateData(bool available);
    void DiplayProgress(int frame, int total);
    void SetFrame(int frameNumber, int total=0);
    void CancelProgress();
    void SaveAsLocalisationLog();
    void SaveAsReport();
    void GetSettings();
    void SetOpenFileList(std::vector<QFileInfo> files);
protected:
    OfflineLocalisation* m_offline_loc;
    QTextBrowser* m_fileListDisplay;
    QProgressDialog* m_progressBar;
    QString m_previous_log_path;

private:
    QStringList GetLogPaths(const QDir& directory);
    void MakeLayout();
    bool m_external_reader;
    LogFileReader* m_reader;
    OfflineLocBatch* m_batch_processsor;
};


#endif // OFFLINELOCALISATIONDIALOG_H
