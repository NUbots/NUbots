#ifndef OFFLINELOCALISATIONDIALOG_H
#define OFFLINELOCALISATIONDIALOG_H

#include <QDialog>
#include <QFileInfo>
#include <vector>
#include <QStringList>
class OfflineLocalisation;
class QTextBrowser;
class QProgressDialog;
class Localisation;
class SelfLocalisation;
class LogFileReader;
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

public slots:
    void OpenLogFiles();
    void BeginSimulation();
    void CompleteSimulation();
    void DiplayProgress(int frame, int total);
    void SetFrame(int frameNumber, int total=0);
    void CancelProgress();
    void SaveAsLocalisationLog();
    void SetOpenFileList(std::vector<QFileInfo> files);
protected:
    OfflineLocalisation* m_offline_loc;
    QTextBrowser* m_fileListDisplay;
    QProgressDialog* m_progressBar;

private:
    void MakeLayout();
    bool m_external_reader;
    LogFileReader* m_reader;
};

#endif // OFFLINELOCALISATIONDIALOG_H
