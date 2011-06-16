#ifndef FRAMEINFORMATIONWIDGET_H
#define FRAMEINFORMATIONWIDGET_H

#include <QWidget>

class QLabel;
class NUImage;
class QGridLayout;

class frameInformationWidget : public QWidget
{
Q_OBJECT
public:
    explicit frameInformationWidget(QWidget *parent = 0);
    ~frameInformationWidget();
signals:

public slots:
    void setFrameSource(QString sourceName);
    void setRawImage(const NUImage* image);
    void setImageResolution(int imageWidth, int imageHeight);
    void setTimestamp(double timestamp);
private:
    QLabel* m_sourceLabel;
    QLabel* m_sourceValueLabel;
    QLabel* m_imageResolutionLabel;
    QLabel* m_imageResolutionValueLabel;
    QLabel* m_timeStampLabel;
    QLabel* m_timeStampValueLabel;
    QGridLayout* m_widgetLayout;
};

#endif // FRAMEINFORMATIONWIDGET_H
