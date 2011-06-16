#ifndef OBJECTDISPLAYWIDGET_H
#define OBJECTDISPLAYWIDGET_H

#include <QWidget>
#include <QTextBrowser>
class FieldObjects;

class ObjectDisplayWidget : public QTextBrowser
{
Q_OBJECT
public:
    explicit ObjectDisplayWidget(QWidget *parent = 0);

signals:

public slots:
    void setObjectData(const FieldObjects* newObjectData);
};

#endif // OBJECTDISPLAYWIDGET_H
