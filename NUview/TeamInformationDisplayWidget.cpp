#include "TeamInformationDisplayWidget.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include <sstream>

TeamInformationDisplayWidget::TeamInformationDisplayWidget(QWidget *parent) :
    QTextBrowser(parent)
{
}

void TeamInformationDisplayWidget::setTeamInfo(const TeamInformation* newTeamInfo)
{
    QString displayText(newTeamInfo->toString().c_str());
    this->setText(displayText);
}
