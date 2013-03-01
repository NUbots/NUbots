#ifndef BATCHSELECTDIALOG_H
#define BATCHSELECTDIALOG_H

#include <QDialog>

class QLineEdit;
class QLabel;
class QComboBox;

class BatchSelectDialog : public QDialog
{
    Q_OBJECT
public slots:
    void selectSourcePath();
    void selectResultPath();
    void verifySourcePath(QString text);
    void verifyResultPath(QString text);

public:
    explicit BatchSelectDialog(QWidget *parent = 0);
    ~BatchSelectDialog();
    QString sourcePath();
    QString resultPath();
    QString experimentType();

private:
    bool verifyAndDisplay(QString text, QLabel* label);
    void makeLayout();
    QComboBox* m_experiment_select;
    QLineEdit* m_source_path_edit;
    QLineEdit* m_result_path_edit;
    QLabel* m_source_path_verified;
    QLabel* m_result_path_verified;
private:
};
#endif // BATCHSELECTDIALOG_H
