#ifndef ROBOTSELECTDIALOG_H
#define ROBOTSELECTDIALOG_H

#include <QDialog>
#include "bonjourrecord.h"
#include <QHostInfo>

class QDialogButtonBox;
class QPushButton;
class QLabel;
class QTreeWidget;
class QTreeWidgetItem;

class BonjourServiceBrowser;
class BonjourServiceResolver;

class robotSelectDialog : public QDialog
{
Q_OBJECT
public:
    robotSelectDialog(QWidget * parent = 0, const QString& service = QString());
    BonjourRecord getBonjourHost(){return m_selectedHost;};
private slots:
    void updateRecords(const QList<BonjourRecord> &list);
    void enableConnectButton();
    void saveSelected();
    void refresh();
private:
    QPushButton *connectButton;
    QPushButton *cancelButton;
    QDialogButtonBox *buttonBox;

    QString m_service;
    QTreeWidget *treeWidget;
    BonjourServiceBrowser *bonjourBrowser;
    BonjourRecord m_selectedHost;
};

#endif // ROBOTSELECTDIALOG_H
