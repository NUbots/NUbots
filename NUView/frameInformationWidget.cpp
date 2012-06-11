#include "frameInformationWidget.h"
#include <QLabel>
#include <QString>
#include <QGridLayout>
#include "Infrastructure/NUImage/NUImage.h"


frameInformationWidget::frameInformationWidget(QWidget *parent) :
    QTextBrowser(parent)
{
    setWindowTitle(tr("Frame Information"));
    setObjectName(tr("Frame Information"));
    QString defaultDisplay = "N/A";

    m_image_width = m_image_height = m_timestamp = m_source = defaultDisplay;
    updateDisplay();
    setMinimumSize(200,100);
}

frameInformationWidget::~frameInformationWidget()
{
}

QSize frameInformationWidget::sizeHint() const
{
    return QSize(200, 100);
}

void frameInformationWidget::updateDisplay()
{
    const QString formatting = "Source: %1\nResolution: %2x%3\nTimestamp: %4 milliseconds";
    QString displayString = formatting.arg(m_source).arg(m_image_width).arg(m_image_height).arg(m_timestamp);
    setText(displayString);
}

void frameInformationWidget::setFrameSource(QString sourceName)
{
    m_source = sourceName;
    updateDisplay();
}

void frameInformationWidget::setRawImage(const NUImage* image)
{
    m_image_width.setNum(image->getWidth());
    m_image_height.setNum(image->getHeight());
    m_timestamp.setNum(image->GetTimestamp());
    updateDisplay();
}

void frameInformationWidget::setImageResolution(int imageWidth, int imageHeight)
{
    m_image_width.setNum(imageWidth);
    m_image_height.setNum(imageHeight);
    updateDisplay();
}

void frameInformationWidget::setTimestamp(double timestamp)
{
    m_timestamp.setNum(timestamp);
    updateDisplay();
}
