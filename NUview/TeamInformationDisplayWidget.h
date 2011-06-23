#ifndef TEAMINFORMATIONDISPLAYWIDGET_H
#define TEAMINFORMATIONDISPLAYWIDGET_H

#include <QWidget>
#include <QTextBrowser>
class TeamInformation;

class TeamInformationDisplayWidget : public QTextBrowser
{
Q_OBJECT
public:
    explicit TeamInformationDisplayWidget(QWidget *parent = 0);

signals:

public slots:
    void setTeamInfo(const TeamInformation* newTeamInfo);
};

#endif // TEAMINFORMATIONDISPLAYWIDGET_H
