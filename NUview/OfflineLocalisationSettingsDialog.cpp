#include "OfflineLocalisationSettingsDialog.h"
#include "ui_OfflineLocalisationSettingsDialog.h"
#include <iostream>

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

void OfflineLocalisationSettingsDialog::on_BranchMethodComboBox_currentIndexChanged(const QString &arg1)
{
    const QString c_none_label = "None";
    QComboBox* prune_combo_box = qFindChild<QComboBox*>(this, "PruneMethodComboBox");

    if(arg1.compare("probabalistic", Qt::CaseInsensitive) == 0)
    {
        // Set to none
        m_previous_id = prune_combo_box->currentIndex();
        prune_combo_box->setEnabled(false);
        prune_combo_box->addItem(c_none_label);
        prune_combo_box->setCurrentIndex(prune_combo_box->findText(c_none_label));
    }
    else
    {
        // Remove none
        prune_combo_box->setEnabled(true);
        int id = prune_combo_box->findText(c_none_label);
        // id = -1 if the text is not found.
        if(id != -1)
        {
            prune_combo_box->removeItem(id);
            prune_combo_box->setCurrentIndex(m_previous_id);
        }
    }

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
    case LocalisationSettings::branch_constraint:
        branch_combo_box->setCurrentIndex(branch_combo_box->findText("Constraint"));
        break;
    case LocalisationSettings::branch_probDataAssoc:
        branch_combo_box->setCurrentIndex(branch_combo_box->findText("Probabalistic"));
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

    if(prune_text.compare("merge", Qt::CaseInsensitive) == 0)
    {
        result.setPruneMethod(LocalisationSettings::prune_merge);
    }
    else if (prune_text.compare("max likelyhood", Qt::CaseInsensitive) == 0)
    {
        result.setPruneMethod((LocalisationSettings::prune_max_likelyhood));
    }
    else if (prune_text.compare("viterbi", Qt::CaseInsensitive) == 0)
    {
        result.setPruneMethod((LocalisationSettings::prune_viterbi));
    }
    else if (prune_text.compare("n-scan", Qt::CaseInsensitive) == 0)
    {
        result.setPruneMethod((LocalisationSettings::prune_nscan));
    }
    else
    {
        result.setPruneMethod((LocalisationSettings::prune_unknown));
    }

    if(branch_text.compare("exhaustive", Qt::CaseInsensitive) == 0)
    {
        result.setBranchMethod(LocalisationSettings::branch_exhaustive);
    }
    else if(branch_text.compare("selective", Qt::CaseInsensitive) == 0)
    {
        result.setBranchMethod(LocalisationSettings::branch_selective);
    }
    else if(branch_text.compare("constraint", Qt::CaseInsensitive) == 0)
    {
        result.setBranchMethod(LocalisationSettings::branch_constraint);
    }
    else if(branch_text.compare("probabalistic", Qt::CaseInsensitive) == 0)
    {
        result.setBranchMethod(LocalisationSettings::branch_probDataAssoc);
    }
    else
    {
        result.setBranchMethod(LocalisationSettings::branch_unknown);
    }
    return result;
}
