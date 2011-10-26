#include "OfflineLocalisationSettingsDialog.h"
#include "ui_OfflineLocalisationSettingsDialog.h"

OfflineLocalisationSettingsDialog::OfflineLocalisationSettingsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::OfflineLocalisationSettingsDialog)
{
    ui->setupUi(this);
    QComboBox* combo_box = qFindChild<QComboBox*>(this, "PruneMethodComboBox");\
    combo_box->setCurrentIndex(-1);
    combo_box->setCurrentIndex(0);
}

OfflineLocalisationSettingsDialog::~OfflineLocalisationSettingsDialog()
{
    delete ui;
}

void OfflineLocalisationSettingsDialog::on_PruneMethodComboBox_currentIndexChanged(const QString &arg1)
{
    QLabel* label = qFindChild<QLabel*>(this, "PruneMethodParamLabel");
    QSpinBox* spin_box = qFindChild<QSpinBox*>(this, "PruneMethodParamSpinBox");

    // Display and additional parameters that may be required.

    if (arg1.toLower() == "viterbi")
    {
        label->show();
        spin_box->show();
        label->setText("Retained = ");
    }
    else if (arg1.toLower() == "n-scan")
    {
        label->show();
        spin_box->show();
        label->setText("N = ");
    }
    else
    {
        label->hide();
        spin_box->hide();
    }
    return;
}

void OfflineLocalisationSettingsDialog::initialiseSettings(const LocalisationSettings& settings)
{
    QComboBox* prune_combo_box = qFindChild<QComboBox*>(this, "PruneMethodComboBox");
    QComboBox* branch_combo_box = qFindChild<QComboBox*>(this, "BranchMethodComboBox");
    switch(settings.branchMethod())
    {
    case LocalisationSettings::branch_exhaustive:
        branch_combo_box->setCurrentIndex(branch_combo_box->findText("Exhaustive"));
        break;
    case LocalisationSettings::branch_selective:
        branch_combo_box->setCurrentIndex(branch_combo_box->findText("Selective"));
        break;
    default:
        branch_combo_box->setCurrentIndex(branch_combo_box->findText("Exhaustive"));
        break;
    }

    switch(settings.pruneMethod())
    {
    case LocalisationSettings::prune_merge:
        prune_combo_box->setCurrentIndex(prune_combo_box->findText("Merge"));
        break;
    case LocalisationSettings::prune_max_likelyhood:
        prune_combo_box->setCurrentIndex(prune_combo_box->findText("Max Likelyhood"));
        break;
    case LocalisationSettings::prune_viterbi:
        prune_combo_box->setCurrentIndex(prune_combo_box->findText("Viterbi"));
        break;
    case LocalisationSettings::prune_nscan:
        prune_combo_box->setCurrentIndex(prune_combo_box->findText("N-Scan"));
        break;
    default:
        prune_combo_box->setCurrentIndex(prune_combo_box->findText("Merge"));
        break;
    }
}

LocalisationSettings OfflineLocalisationSettingsDialog::settings()
{
    LocalisationSettings result;
    QComboBox* prune_combo_box = qFindChild<QComboBox*>(this, "PruneMethodComboBox");
    QComboBox* branch_combo_box = qFindChild<QComboBox*>(this, "BranchMethodComboBox");

    QString prune_text = prune_combo_box->currentText().toLower();
    QString branch_text = branch_combo_box->currentText().toLower();

    if(prune_text == "merge")
    {
        result.setPruneMethod(LocalisationSettings::prune_merge);
    }
    else if (prune_text == "max likelyhood")
    {
        result.setPruneMethod((LocalisationSettings::prune_max_likelyhood));
    }
    else if (prune_text == "viterbi")
    {
        result.setPruneMethod((LocalisationSettings::prune_viterbi));
    }
    else if (prune_text == "n-scan")
    {
        result.setPruneMethod((LocalisationSettings::prune_nscan));
    }
    else
    {
        result.setPruneMethod((LocalisationSettings::prune_unknown));
    }

    if(branch_text == "exhaustive")
    {
        result.setBranchMethod(LocalisationSettings::branch_exhaustive);
    }
    else if(branch_text == "selective")
    {
        result.setBranchMethod(LocalisationSettings::branch_selective);
    }
    else
    {
        result.setBranchMethod(LocalisationSettings::branch_unknown);
    }

    return result;
}
