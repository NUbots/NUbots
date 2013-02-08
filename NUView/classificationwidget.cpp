#include "classificationwidget.h"
#include <QPixmap>
#include <QComboBox>
#include <QSpinBox>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QWidget>
#include <QPushButton>
#include <QFileDialog>
#include <QTimer>
#include <QMainWindow>
#include <QTextStream>
#include "ColorModelConversions.h"
#include "Vision/VisionTools/classificationcolours.h"

ClassificationWidget::ClassificationWidget(QWidget* parent) : QDockWidget(parent)
{
    setObjectName(tr("Classification"));
    setWindowTitle(tr("Classification"));

    // Label for colour label selection drop down box
    colourLabel = new QLabel("Colour Label: ");

    // Drop down box
    coloursComboBox = new QComboBox;

    QPixmap temp_pixmap(22,22);
    unsigned char tempr, tempg, tempb;

    for (int col = 0; col < Vision::num_colours; col++)
    {
        Vision::getColourAsRGB(Vision::getColourFromIndex(col), tempr, tempg, tempb);
        temp_pixmap.fill(QColor(tempr, tempg, tempb));
        coloursComboBox->addItem(QIcon(temp_pixmap), tr(Vision::getColourName(Vision::getColourFromIndex(col)).c_str()));
    }

    // Colour selection group
    autoSoftColourCheckBox = new QCheckBox("Auto Soft Colour");
    colourSelectLayout = new QHBoxLayout;
    //colourSelectLayout->addWidget(colourLabel);
    colourSelectLayout->addWidget(coloursComboBox,1);
    colourSelectLayout->addWidget(autoSoftColourCheckBox,0);


    // Selected Colour
    selectedColourLabel = new QLabel("Selected Colour: ");
    selectedColourLayout = new QHBoxLayout;
    //selectedColourLayout->addWidget(selectedColourLabel,0);

    colourPreviewLabel = new QLabel("");
    temp_pixmap.fill(Qt::black);
    colourPreviewLabel->setPixmap(temp_pixmap);
    selectedColourLayout->addWidget(colourPreviewLabel);

    for(int channel = 0; channel < numChannels; channel++)
    {
        channelSelectors[channel] = new QSpinBox;
        channelSelectors[channel]->setRange(0,255);
        channelSelectors[channel]->setValue(0);
        selectedColourLayout->addWidget(channelSelectors[channel],1);
    }

    // Colour Range Selections
    allValuesLabel = new QLabel("All Bounds");
    allValuesSlider = new QSlider(Qt::Horizontal);
    allValuesSlider->setMinimum(0);
    allValuesSlider->setMaximum(20);
    allValuesSlider->setValue(5);
    boundaryGroupBox = new QGroupBox(tr("Selection Boundaries"));
    boundaryLayout = new QGridLayout;
    boundaryLayout->setColumnStretch(1,1);
    boundaryLayout->setColumnStretch(2,1);

    minLabel = new QLabel(tr("Min"));
    maxLabel = new QLabel(tr("Max"));
    boundaryLayout->addWidget(allValuesLabel,0,1);
    boundaryLayout->addWidget(allValuesSlider,0,2);
    boundaryLayout->addWidget(minLabel,1,1);
    boundaryLayout->addWidget(maxLabel,1,2);

    for(int channel = 0; channel < numChannels; channel++)
    {
        channelLabels[channel] = new QLabel(tr(""));
        channelMinSelectors[channel] = new QSpinBox;
        channelMinSelectors[channel]->setRange(-255, 0);
        channelMinSelectors[channel]->setValue(-5);
        channelMaxSelectors[channel] = new QSpinBox;
        channelMaxSelectors[channel]->setRange(0, 255);
        channelMaxSelectors[channel]->setValue(5);
        boundaryLayout->addWidget(channelLabels[channel],channel+2,0);
        boundaryLayout->addWidget(channelMinSelectors[channel],channel+2,1);
        boundaryLayout->addWidget(channelMaxSelectors[channel],channel+2,2);
    }
    boundaryGroupBox->setLayout(boundaryLayout);

    // Colour space selection.
    colourSpaceLabel = new QLabel(tr("Colour Space: "));
    colourSpaceComboBox = new QComboBox;
    colourSpaceComboBox->addItem(tr("YCbCr"), 0);
//TODO: Disabled until conversions are sorted
    colourSpaceComboBox->addItem(tr("HSV"), 1);
    colourSpaceComboBox->addItem(tr("RGB"), 2);

    colourSpaceSelectLayout = new QHBoxLayout;
    colourSpaceSelectLayout->addWidget(colourSpaceLabel,0);
    colourSpaceSelectLayout->addWidget(colourSpaceComboBox,1);

    // Buttons
    openFileButton = new QPushButton("&Open",this);
    saveAsFileButton = new QPushButton("Save &As",this);
    FileButtonLayout = new QHBoxLayout;
    FileButtonLayout->addWidget(openFileButton, 1);
    FileButtonLayout->addWidget(saveAsFileButton, 1);

    // Selected Colour statistics:
    StatisticsLayout = new QGridLayout;
    for (int col = 0; col < Vision::num_colours; col++)
    {
        ColourLabel[col] = new QLabel(tr(Vision::getColourName(Vision::getColourFromIndex(col)).c_str()));
        PercentageSelectedLabel[col] = new QLabel(tr("0 %"));
        PixelSelectedLabelOverlapped[col] = new QLabel(tr("0 Colours"));
        StatisticsLayout->addWidget(ColourLabel[col],col,1);
        StatisticsLayout->addWidget(PixelSelectedLabelOverlapped[col],col,2);
        StatisticsLayout->addWidget(PercentageSelectedLabel[col],col,3);
    }
    StatisticsGroupBox = new QGroupBox(tr("OverLapping Colour Statistics"));
    StatisticsGroupBox->setCheckable(true);
    StatisticsGroupBox->setLayout(StatisticsLayout);
    connect(StatisticsGroupBox,SIGNAL(toggled(bool)),this,SLOT(setOverlapVisible(bool)));
    StatisticsGroupBox->setChecked(false);

    groupLayout = new QVBoxLayout;
    groupLayout->addLayout(selectedColourLayout);
    groupLayout->addLayout(colourSelectLayout);
    groupLayout->addWidget(boundaryGroupBox);
    groupLayout->addLayout(colourSpaceSelectLayout);
    groupLayout->addLayout(FileButtonLayout);
    groupLayout->addWidget(StatisticsGroupBox);
    groupLayout->addStretch();

    window = new QWidget;
    window->setLayout(groupLayout);
    setWidget(window);

    // Setup signals
    for(int channel = 0; channel < numChannels; channel++)
    {
        connect(channelSelectors[channel], SIGNAL(valueChanged(int)), this, SLOT(drawSelectedColour()));

        // Setup selection changed signals
        connect(channelSelectors[channel], SIGNAL(valueChanged(int)), this, SIGNAL(selectionChanged()));
        connect(channelMinSelectors[channel], SIGNAL(valueChanged(int)), this, SIGNAL(selectionChanged()));
        connect(channelMaxSelectors[channel], SIGNAL(valueChanged(int)), this, SIGNAL(selectionChanged()));
    }
    connect(colourSpaceComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(setColourSpace(int)));

    connect(colourSpaceComboBox, SIGNAL(currentIndexChanged(int)), this, SIGNAL(selectionChanged()));
    connect(coloursComboBox, SIGNAL(currentIndexChanged(int)), this, SIGNAL(selectionChanged()));
    connect(autoSoftColourCheckBox,SIGNAL(stateChanged(int)), this, SLOT(autoSoftColourStateChanged(int)));

    connect(openFileButton, SIGNAL(clicked()), this, SLOT(doOpen()));
    connect(saveAsFileButton, SIGNAL(clicked()), this, SLOT(doSaveAs()));

    connect(allValuesSlider, SIGNAL(sliderMoved(int)), this, SLOT(setAllBoundaries(int)));
    // Set inital colour space
    currentColourSpace = YCbCr;
    colourSpaceComboBox->setCurrentIndex(currentColourSpace);
    setColourSpace(currentColourSpace);

    // Start autosave timer.
    autosaveTimer = new QTimer(this);
    connect( autosaveTimer, SIGNAL(timeout()), this, SLOT(PerformAutosave()) );
    int autosavePeriodMinutes = 2;
    autosaveTimer->start(autosavePeriodMinutes * 60 * 1000);

}

ClassificationWidget::~ClassificationWidget()
{
    // Delete Layouts
    delete colourSpaceSelectLayout;
    delete boundaryLayout;
    delete selectedColourLayout;
    delete colourSelectLayout;

    // Delete Labels
    delete colourLabel;
    delete colourPreviewLabel;
    delete selectedColourLabel;
    delete minLabel;
    delete maxLabel;
    delete colourSpaceLabel;
    for(int channel = 0; channel < numChannels; channel++)
    {
        delete channelLabels[channel];
    }

    // Delete Controls
    delete coloursComboBox;
    delete colourSpaceComboBox;
    delete autoSoftColourCheckBox;
    for(int channel = 0; channel < numChannels; channel++)
    {
        delete channelSelectors[channel];
        delete channelMinSelectors[channel];
        delete channelMaxSelectors[channel];
    }

    // Delete Containers
    delete groupLayout;
    delete boundaryGroupBox;
    delete window;
    return;
}

void ClassificationWidget::setOverlapVisible(bool isVisible)
{
    QLayoutItem *item = 0;
    QWidget *widget = 0;
    QGridLayout *layout = StatisticsLayout;
    for(int i = 0; i < layout->rowCount(); ++i)
    {
        for(int j = 0; j < layout->columnCount(); ++j)
        {
            item = layout->itemAtPosition(i,j);
            widget=item?item->widget():0;
            if(widget)
                widget->setVisible(isVisible);
        }
    }
}

void ClassificationWidget::autoSoftColourStateChanged(int newState){
    emit autoSoftColourChanged(newState == Qt::Checked);
}

void ClassificationWidget::doOpen()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                tr("Open Lookup Table"), ".",
                                tr("Lookup Table Files (*.lut)"));

    if (!fileName.isEmpty()){
        emit openLookupTableFile(fileName);
    }
    QString message = "Lookup Table Loaded: ";
    message.append(fileName);
    emit displayStatusBarMessage(message, 3000);
}

void ClassificationWidget::doSaveAs()
{
    QString fileName = QFileDialog::getSaveFileName(this,
                                tr("Save Lookup Table"), tr("default.lut"),
                                tr("Lookup Table Files (*.lut)"));

    if (!fileName.isEmpty()){
        emit saveLookupTableFile(fileName);
        QString message = "Saving Lookup Table: ";
        message.append(fileName);
        emit displayStatusBarMessage(message, 3000);
    }
}

void ClassificationWidget::PerformAutosave()
{
    emit saveLookupTableFile(QString("autosave.lut"));
    QString message = "Performing Lookup Table Autosave...";
    emit displayStatusBarMessage(message, 3000);
}

Pixel ClassificationWidget::getCurrentColour()
{    
    Pixel currentColour =   convertToYCbCr( channelSelectors[0]->value(),
                                            channelSelectors[1]->value(),
                                            channelSelectors[2]->value(),
                                            getCurrentColourSpace());
    return currentColour;
}

std::vector<Pixel> ClassificationWidget::getSelectedColours()
{
    std::vector<Pixel> result;
    ColourSpace currentSpace = getCurrentColourSpace();
    Pixel tempColour;
    int channel[3];
    channel[0] = channelSelectors[0]->value();
    channel[1] = channelSelectors[1]->value();
    channel[2] = channelSelectors[2]->value();

    int min[3];
    int max[3];
    for (int chan = 0; chan < numChannels; chan++)
    {
        min[chan] = channel[chan] + channelMinSelectors[chan]->value();
        max[chan] = channel[chan] + channelMaxSelectors[chan]->value();
    }

    for (int chan0 = min[0]; chan0 <= max[0]; chan0++)
    {
        if((chan0 < 0) || (chan0 > 255)) continue;
        for (int chan1 = min[1]; chan1 <= max[1]; chan1++)
        {
            if((chan1 < 0) || (chan1 > 255)) continue;
            for (int chan2 = min[2]; chan2 <= max[2]; chan2++)
            {
                if((chan2 < 0) || (chan2 > 255)) continue;
                tempColour = convertToYCbCr(chan0, chan1, chan2, currentSpace);
                result.push_back(tempColour);
            }
        }
    }
    return result;
}

Vision::Colour ClassificationWidget::getColourLabel()
{
    return Vision::getColourFromIndex(coloursComboBox->currentIndex());
}

// Slots
void ClassificationWidget::setColourSpace(int newColourSpaceIndex)
{
    Pixel currColour = getCurrentColour();
    switch(newColourSpaceIndex)
    {
        case YCbCr:
            channelLabels[0]->setText(tr("Y: "));
            channelLabels[1]->setText(tr("Cb: "));
            channelLabels[2]->setText(tr("Cr: "));
            currentColourSpace = YCbCr;
            break;
        case HSV:
            channelLabels[0]->setText(tr("Hue: "));
            channelLabels[1]->setText(tr("Saturation: "));
            channelLabels[2]->setText(tr("Value: "));
            // Convert the selected value to the new colour model if required.
            currentColourSpace = HSV;
            break;
        case RGB:
            channelLabels[0]->setText(tr("Red: "));
            channelLabels[1]->setText(tr("Green: "));
            channelLabels[2]->setText(tr("Blue: "));
            // Convert the selected value to the new colour model if required.
            currentColourSpace = RGB;
            break;
        default:
            break;
    };

    setColour(currColour);
}

void ClassificationWidget::drawSelectedColour()
{
    Pixel drawingColour = getCurrentColour();
    unsigned char temp[3];
    ColorModelConversions::fromYCbCrToRGB(drawingColour.y,drawingColour.cb,drawingColour.cr,temp[0],temp[1],temp[2]);
    QPixmap temp_pixmap(22,22);
    temp_pixmap.fill(QColor(temp[0], temp[1], temp[2]));
    colourPreviewLabel->setPixmap(temp_pixmap);
}

void ClassificationWidget::setColour(Pixel colour)
{
    // Disable the signals that update the colour selection, otherwise weird things happen.
    for(int channel = 0; channel < numChannels; channel++)
    {
        channelSelectors[channel]->blockSignals(true);
    }
    unsigned char temp[3];
    convertfromYCbCr(colour, temp[0], temp[1], temp[2], getCurrentColourSpace());
    channelSelectors[0]->setValue(temp[0]);
    channelSelectors[1]->setValue(temp[1]);
    channelSelectors[2]->setValue(temp[2]);

    // Enable the signals that update the colour selection
    for(int channel = 0; channel < numChannels; channel++)
    {
        channelSelectors[channel]->blockSignals(false);
    }
    drawSelectedColour(); // Now force a redraw of the newly selected colour.
    emit selectionChanged(); // Signal that the colour selection has changed.
}

void ClassificationWidget::setAllBoundaries( int newBound)
{
    for(int channel = 0; channel < numChannels; channel++)
    {
        channelMinSelectors[channel]->setValue(-newBound);
        channelMaxSelectors[channel]->setValue(newBound);
    }
}

Pixel ClassificationWidget::convertToYCbCr(unsigned char chan0,unsigned char chan1,unsigned char chan2, ColourSpace originalColourSpace)
{
    Pixel result;
    switch(originalColourSpace)
    {
        case YCbCr:
            result.y = chan0;
            result.cb = chan1;
            result.cr = chan2;
            break;
        case HSV:
            ColorModelConversions::fromHSVToYCbCr(chan0,chan1,chan2,result.y,result.cb,result.cr);
            break;
        case RGB:
            ColorModelConversions::fromRGBToYCbCr(chan0,chan1,chan2,result.y,result.cb,result.cr);
            break;
    }
    return result;
}

void ClassificationWidget::convertfromYCbCr(Pixel colour, unsigned char& chan0, unsigned char& chan1, unsigned char& chan2, ColourSpace newColourSpace)
{
    switch(newColourSpace){
        case YCbCr:
            chan0 = colour.y;
            chan1 = colour.cb;
            chan2 = colour.cr;
            break;
        case HSV:
            ColorModelConversions::fromYCbCrToHSV(colour.y, colour.cb, colour.cr, chan0, chan1, chan2);
            break;
        case RGB:
            ColorModelConversions::fromYCbCrToRGB(colour.y, colour.cb, colour.cr, chan0, chan1, chan2);
            break;
    }
}

void ClassificationWidget::updateStatistics(float* PixelsOverLapped)
{
    //qDebug() << "Updating Stats:" ;
    float total = 0;
    for (int col = 0; col < Vision::num_colours; col++)
    {
        total = total + PixelsOverLapped[col];
    }
    for (int col = 0; col < Vision::num_colours; col++)
    {
        QString result;
        QString pixelsOverlapped;
        QTextStream(&result) << PixelsOverLapped[col]/total*100 << " %";
        QTextStream(&pixelsOverlapped) << PixelsOverLapped[col] << " Colours";
        PercentageSelectedLabel[col]->setText(result);
        PixelSelectedLabelOverlapped[col]->setText(pixelsOverlapped);
    }

}
