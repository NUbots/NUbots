#ifndef OFFLINELOCALISATIONSETTINGSDIALOG_H
#define OFFLINELOCALISATIONSETTINGSDIALOG_H

#include <QDialog>
#include "Localisation/LocalisationSettings.h"

namespace Ui {
    class OfflineLocalisationSettingsDialog;
}

class OfflineLocalisationSettingsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit OfflineLocalisationSettingsDialog(QWidget *parent = 0);
    ~OfflineLocalisationSettingsDialog();
    void initialiseSettings(const LocalisationSettings& settings);
    LocalisationSettings settings();

private slots:
    void on_PruneMethodComboBox_currentIndexChanged(const QString &arg1);

private:
    Ui::OfflineLocalisationSettingsDialog *ui;
};
#endif // OFFLINELOCALISATIONSETTINGSDIALOG_H
