#ifndef ROBOTSELECTDIALOG_H
#define ROBOTSELECTDIALOG_H

class BonjourProvider;
class NUHostInfo;

#include <QDialog>
#include <QHostInfo>
#include <string>
#include <list>
using namespace std;

class QDialogButtonBox;
class QPushButton;
class QLabel;
class QTreeWidget;
class QTreeWidgetItem;

class RobotSelectDialog : public QDialog
{
Q_OBJECT
public:
    RobotSelectDialog(QWidget* parent = 0, BonjourProvider* provider = 0);
    ~RobotSelectDialog();
    
    NUHostInfo getSelectedHost();
    vector<NUHostInfo>& getSelectedHosts();
    
private slots:
    void populateTree();
    void saveSelected();
private:
    void addService(const string& service, list<NUHostInfo>& hosts);
    
    void enableConnectButton();
private:
    BonjourProvider* m_bonjour;					//!< a pointer to the bonjour provider
    vector<NUHostInfo> m_selected_hosts;		//!< a list of the currently selected hosts
    
    QPushButton* m_connect_button;
    QPushButton* m_cancel_button;
    QDialogButtonBox* m_button_box;
    QTreeWidget* m_tree;
};

#endif // ROBOTSELECTDIALOG_H
