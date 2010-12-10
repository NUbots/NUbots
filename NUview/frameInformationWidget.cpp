#include "frameInformationWidget.h"
#include <QLabel>
#include <QString>
#include <QGridLayout>
#include "Infrastructure/NUImage/NUImage.h"


frameInformationWidget::frameInformationWidget(QWidget *parent) :
    QWidget(parent)
{
    setWindowTitle(tr("Frame Information"));
    setObjectName(tr("Frame Information"));
    m_widgetLayout = new QGridLayout();

    m_sourceLabel = new QLabel(tr("Source: "));
    m_sourceValueLabel = new QLabel(tr("N/A"));

    m_imageResolutionLabel = new QLabel(tr("Image Resolution: "));
    m_imageResolutionValueLabel = new QLabel(tr("N/A"));

    m_timeStampLabel = new QLabel(tr("Timestamp: "));
    m_timeStampValueLabel = new QLabel(tr("N/A"));

    m_widgetLayout->addWidget(m_sourceLabel,0,0,Qt::AlignRight);
    m_widgetLayout->addWidget(m_sourceValueLabel,0,1,Qt::AlignLeft);

    m_widgetLayout->addWidget(m_imageResolutionLabel,1,0,Qt::AlignRight);
    m_widgetLayout->addWidget(m_imageResolutionValueLabel,1,1,Qt::AlignLeft);

    m_widgetLayout->addWidget(m_timeStampLabel,2,0,Qt::AlignRight);
    m_widgetLayout->addWidget(m_timeStampValueLabel,2,1,Qt::AlignLeft);
    setLayout(m_widgetLayout);
}

frameInformationWidget::~frameInformationWidget()
{
    delete m_widgetLayout;
}

void frameInformationWidget::setFrameSource(QString sourceName)
{
    m_sourceValueLabel->setText(sourceName);
}


void frameInformationWidget::setRawImage(const NUImage* image)
{
    setImageResolution(image->getWidth(), image->getHeight());
    setTimestamp(image->m_timestamp);
}

void frameInformationWidget::setImageResolution(int imageWidth, int imageHeight)
{
    QString resolutionMessage(tr("%1x%2 pixels"));
    m_imageResolutionValueLabel->setText(resolutionMessage.arg(imageWidth).arg(imageHeight));
}

void frameInformationWidget::setTimestamp(double timestamp)
{
    QString timestampMessage(tr("%1 milliseconds"));
    m_timeStampValueLabel->setText(timestampMessage.arg(timestamp));
}
