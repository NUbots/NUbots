#include "BatchSelectDialog.h"
#include <QLineEdit>
#include <QLabel>
#include <QComboBox>
#include <QDir>
#include <QFileDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QDialogButtonBox>


BatchSelectDialog::BatchSelectDialog(QWidget *parent): QDialog(parent)
{
    makeLayout();
}

BatchSelectDialog::~BatchSelectDialog()
{
}

QString BatchSelectDialog::sourcePath()
{
    QString dir = m_source_path_edit->text();
    if(QDir(dir).exists())
    {
        return dir;
    }
    return QString();
}

QString BatchSelectDialog::resultPath()
{
    QString dir = m_result_path_edit->text();
    if(QDir(dir).exists())
    {
        return dir;
    }
    return QString();
}

void BatchSelectDialog::selectSourcePath()
{
    QString dir = QFileDialog::getExistingDirectory(this, tr("Select Source Directory"), m_source_path_edit->text());
    if(QDir(dir).exists())
    {
        m_source_path_edit->setText(dir);
    }
}

void BatchSelectDialog::selectResultPath()
{
    QString dir = QFileDialog::getExistingDirectory(this, tr("Select Result Directory"), m_result_path_edit->text());
    if(QDir(dir).exists())
    {
        m_result_path_edit->setText(dir);
    }
}

void BatchSelectDialog::verifySourcePath(QString text)
{
    bool valid = verifyAndDisplay(text, m_source_path_verified);
    if(valid and !m_source_path_edit->text().isEmpty() and m_result_path_edit->text() == QDir::currentPath())
    {
        m_result_path_edit->setText(m_source_path_edit->text());
    }
}

void BatchSelectDialog::verifyResultPath(QString text)
{
    verifyAndDisplay(text, m_result_path_verified);
}

bool BatchSelectDialog::verifyAndDisplay(QString text, QLabel* label)
{
    bool result;
    if(QDir(text).exists())
    {
        result = true;
        label->setPixmap(QPixmap(":/icons/check.png"));
    }
    else
    {
        result = false;
        label->setPixmap(QPixmap(":/icons/cross.png"));
    }
    return result;
}

QString BatchSelectDialog::experimentType()
{
    return m_experiment_select->currentText();
}

void BatchSelectDialog::makeLayout()
{
    QVBoxLayout *dialog_layout = new QVBoxLayout();

    this->setWindowTitle("Batch Selection");

    // Experiment Settings
    QHBoxLayout *experiment_settings_layout = new QHBoxLayout();
    m_experiment_select = new QComboBox();
    // Add experiment types
    m_experiment_select->addItem("Standard");
    m_experiment_select->addItem("Multiple Model Methods");
    m_experiment_select->addItem("Filter Type Comparison");
    m_experiment_select->setCurrentIndex(0);

    experiment_settings_layout->addWidget(new QLabel("Experiment Type: "));
    experiment_settings_layout->addWidget(m_experiment_select, 1);
    dialog_layout->addLayout(experiment_settings_layout);


    // Source path selection
    QGroupBox *source_path_box = new QGroupBox("Source Directory");
    QHBoxLayout *source_path_layout = new QHBoxLayout(source_path_box);
    m_source_path_edit = new QLineEdit();
    m_source_path_edit->setMinimumWidth(300);
    connect(m_source_path_edit, SIGNAL(textChanged(QString)), this, SLOT(verifySourcePath(QString)));
    source_path_layout->addWidget(m_source_path_edit);

    m_source_path_verified = new QLabel();
    source_path_layout->addWidget(m_source_path_verified);

    QPushButton *source_path_browse_button = new QPushButton("Browse...");
    connect(source_path_browse_button, SIGNAL(clicked()), this, SLOT(selectSourcePath()));
    source_path_layout->addWidget(source_path_browse_button);
    source_path_box->setLayout(source_path_layout);
    dialog_layout->addWidget(source_path_box);

    // Results path select
    QGroupBox *result_path_box = new QGroupBox("Results Directory");
    QHBoxLayout *result_path_layout = new QHBoxLayout(result_path_box);
    m_result_path_edit = new QLineEdit();
    m_result_path_edit->setMinimumWidth(300);
    connect(m_result_path_edit, SIGNAL(textChanged(QString)), this, SLOT(verifyResultPath(QString)));
    result_path_layout->addWidget(m_result_path_edit);

    m_result_path_verified = new QLabel();
    result_path_layout->addWidget(m_result_path_verified);

    QPushButton *result_path_browse_button = new QPushButton("Browse...");
    connect(result_path_browse_button, SIGNAL(clicked()), this, SLOT(selectResultPath()));
    result_path_layout->addWidget(result_path_browse_button);
    result_path_box->setLayout(result_path_layout);
    dialog_layout->addWidget(result_path_box);

    // Add the dialog buttons
    QDialogButtonBox* dlg_button_box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(dlg_button_box, SIGNAL(accepted()), this, SLOT(accept()));
    connect(dlg_button_box, SIGNAL(rejected()), this, SLOT(reject()));
    dialog_layout->addWidget(dlg_button_box);

    m_source_path_edit->setText(QDir::currentPath());
    m_result_path_edit->setText(QDir::currentPath());

    setLayout(dialog_layout);
}
