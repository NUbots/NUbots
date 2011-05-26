#ifndef OFFLINELOCALISATIONDIALOG_H
#define OFFLINELOCALISATIONDIALOG_H

#include <QDialog>
class OfflineLocalisation;
class QTextBrowser;
class QProgressDialog;
class Localisation;
class OfflineLocalisationDialog : public QDialog
{
    Q_OBJECT
public:
    explicit OfflineLocalisationDialog(QWidget *parent = 0);
    ~OfflineLocalisationDialog();

signals:
    void LocalisationChanged(const Localisation*);

public slots:
    void OpenLogFiles();
    void BeginSimulation();
    void CompleteSimulation();
    void DiplayProgress(int frame, int total);
    void SetFrame(int frameNumber, int total=0);
    void CancelProgress();
protected:
    OfflineLocalisation* m_offline_loc;
    QTextBrowser* fileListDisplay;
    QProgressDialog* m_progressBar;
};

#endif // OFFLINELOCALISATIONDIALOG_H
