#ifndef ROBOTSELECTDIALOG_H
#define ROBOTSELECTDIALOG_H

#include <QDialog>
#include "bonjourrecord.h"

class QDialogButtonBox;
class QPushButton;
class QLabel;
class QTreeWidget;

class BonjourServiceBrowser;

class robotSelectDialog : public QDialog
{
Q_OBJECT
public:
    robotSelectDialog(QWidget * parent = 0);

private slots:
    void updateRecords(const QList<BonjourRecord> &list);
    void enableConnectButton();

private:
    QPushButton *connectButton;
    QPushButton *cancelButton;
    QDialogButtonBox *buttonBox;

    QTreeWidget *treeWidget;

    BonjourServiceBrowser *bonjourBrowser;
};

#endif // ROBOTSELECTDIALOG_H
