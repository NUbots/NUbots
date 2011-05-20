#ifndef GAMEINFORMATIONDISPLAYWIDGET_H
#define GAMEINFORMATIONDISPLAYWIDGET_H

#include <QWidget>
#include <QTextBrowser>
class GameInformation;

class GameInformationDisplayWidget : public QTextBrowser
{
Q_OBJECT
public:
    explicit GameInformationDisplayWidget(QWidget *parent = 0);

signals:

public slots:
    void setGameInfo(const GameInformation* newGameInfoData);
};

#endif // GAMEINFORMATIONDISPLAYWIDGET_H
