#ifndef FRAMEINFORMATIONWIDGET_H
#define FRAMEINFORMATIONWIDGET_H

#include <QTextBrowser>

class QLabel;
class NUImage;
class QGridLayout;

class frameInformationWidget : public QTextBrowser
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
    void updateDisplay();
    QSize sizeHint() const;
private:
    QString m_image_width;
    QString m_image_height;
    QString m_timestamp;
    QString m_source;
};

#endif // FRAMEINFORMATIONWIDGET_H
