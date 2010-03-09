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
#include "ColorModelConversions.h"
#include "Vision/ClassificationColours.h"
#include "Tools/Math/General.h"

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

    for (int col = 0; col < ClassIndex::num_colours; col++)
    {
        ClassIndex::getColourIndexAsRGB(col, tempr, tempg, tempb);
        temp_pixmap.fill(QColor(tempr, tempg, tempb));
        coloursComboBox->addItem(QIcon(temp_pixmap), tr(ClassIndex::getColourNameFromIndex(col)));
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
    allValuesSlider->setMaximum(30);
    allValuesSlider->setValue(10);
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
        channelMinSelectors[channel]->setValue(-10);
        channelMaxSelectors[channel] = new QSpinBox;
        channelMaxSelectors[channel]->setRange(0, 255);
        channelMaxSelectors[channel]->setValue(10);
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

    groupLayout = new QVBoxLayout;
    groupLayout->addLayout(selectedColourLayout);
    groupLayout->addLayout(colourSelectLayout);
    groupLayout->addWidget(boundaryGroupBox);
    groupLayout->addLayout(colourSpaceSelectLayout);
    groupLayout->addLayout(FileButtonLayout);
    groupLayout->addStretch();

    window = new QWidget;
    window->setLayout(groupLayout);
    setWidget(window);

    // Setup signals
    for(int channel = 0; channel < numChannels; channel++)
    {
        connect(channelSelectors[channel], SIGNAL(valueChanged(int)), this, SLOT(drawSelectedColour()));

        // Setup selection changed signals
        connect(channelSelectors[channel], SIGNAL(valueChanged(int)), this, SLOT(selectionChanged()));
        connect(channelMinSelectors[channel], SIGNAL(valueChanged(int)), this, SLOT(selectionChanged()));
        connect(channelMaxSelectors[channel], SIGNAL(valueChanged(int)), this, SLOT(selectionChanged()));
    }
    connect(colourSpaceComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(updateColourSpace(int)));

    connect(colourSpaceComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(selectionChanged()));
    connect(coloursComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(selectionChanged()));
    connect(autoSoftColourCheckBox,SIGNAL(stateChanged(int)), this, SLOT(autoSoftColourStateChanged(int)));

    connect(openFileButton, SIGNAL(clicked()), this, SLOT(doOpen()));
    connect(saveAsFileButton, SIGNAL(clicked()), this, SLOT(doSaveAs()));

    connect(allValuesSlider, SIGNAL(sliderMoved(int)), this, SLOT(setAllBoundaries(int)));
    // Set inital colour space
    currentColourSpace = YCbCr;
    colourSpaceComboBox->setCurrentIndex(currentColourSpace);
    updateColourSpace(currentColourSpace);

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

void ClassificationWidget::autoSoftColourStateChanged(int newState){
    emit autoSoftColourChanged(newState == Qt::Checked);
}


void ClassificationWidget::selectionChanged()
{
    emit newSelection();
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

pixels::Pixel ClassificationWidget::getCurrentColour()
{
    ColourSpace currentSpace = getCurrentColourSpace();
    pixels::Pixel currentColour;
    switch(currentSpace){
        case YCbCr:
            currentColour.y = channelSelectors[0]->value();
            currentColour.cb = channelSelectors[1]->value();
            currentColour.cr = channelSelectors[2]->value();
            break;
        case HSV:
            currentColour.h = channelSelectors[0]->value();
            currentColour.s = channelSelectors[1]->value();
            currentColour.v = channelSelectors[2]->value();
            break;
        case RGB:
            currentColour.r = channelSelectors[0]->value();
            currentColour.g = channelSelectors[1]->value();
            currentColour.b = channelSelectors[2]->value();
            break;
    }
    return currentColour;
}

pixels::Pixel ClassificationWidget::convertToColourSpace(pixels::Pixel source, int sourceColourSpace, int desiredColourSpace)
{
    pixels::Pixel result = source;
    if(sourceColourSpace == desiredColourSpace) return result;

    switch(sourceColourSpace){
        case YCbCr:
            switch(desiredColourSpace){
                case YCbCr:
                    // No conversion needed
                    break;
                case HSV:
                    ColorModelConversions::fromYCbCrToHSV(  source.y,
                                                            source.cb,
                                                            source.cr,
                                                            result.h,
                                                            result.s,
                                                            result.v);
                    break;
                case RGB:
                    ColorModelConversions::fromYCbCrToRGB(  source.y,
                                                            source.cb,
                                                            source.cr,
                                                            result.r,
                                                            result.g,
                                                            result.b);
                    break;
            };
            break;
        case HSV:
            switch(desiredColourSpace){
                case YCbCr:
                    ColorModelConversions::fromHSVToYCbCr(  source.h,
                                                            source.s,
                                                            source.v,
                                                            result.y,
                                                            result.cb,
                                                            result.cr);
                    break;
                case HSV:
                    // No conversion required
                    break;
                case RGB:
                    ColorModelConversions::fromHSVToRGB(    source.h,
                                                            source.s,
                                                            source.v,
                                                            result.r,
                                                            result.g,
                                                            result.b);
                    break;
            };
            break;
        case RGB:
            switch(desiredColourSpace){
                case YCbCr:
                    ColorModelConversions::fromRGBToYCbCr(  source.r,
                                                            source.g,
                                                            source.b,
                                                            result.y,
                                                            result.cb,
                                                            result.cr);
                    break;
                case HSV:
                    ColorModelConversions::fromRGBToHSV(    source.r,
                                                            source.g,
                                                            source.b,
                                                            result.h,
                                                            result.s,
                                                            result.v);
                    break;
                case RGB:
                    // No conversion required
                    break;
            };
            break;
    };
    return result;
}


pixels::Pixel ClassificationWidget::getCurrentColourInColourModel(int desiredColourSpace)
{
    ColourSpace currentSpace = getCurrentColourSpace();
    pixels::Pixel currentColour = getCurrentColour();
    return convertToColourSpace(currentColour, currentSpace, desiredColourSpace);
}

std::vector<pixels::Pixel> ClassificationWidget::getSelectedColours(int desiredColourSpace)
{
    std::vector<pixels::Pixel> result;
    ColourSpace currentSpace = getCurrentColourSpace();
    pixels::Pixel currentColour = getCurrentColour();
    pixels::Pixel tempColour = currentColour;

    int channel[3];
    switch(currentSpace)
    {
        case YCbCr:
            channel[0] = currentColour.y;
            channel[1] = currentColour.cb;
            channel[2] = currentColour.cr;
            break;
        case HSV:
            channel[0] = currentColour.h;
            channel[1] = currentColour.s;
            channel[2] = currentColour.v;
            break;
        case RGB:
            channel[0] = currentColour.r;
            channel[1] = currentColour.g;
            channel[2] = currentColour.b;
            break;
        default:
            channel[0] = 0;
            channel[1] = 0;
            channel[2] = 0;
            break;
    }

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
                switch(currentSpace)
                {
                    case YCbCr:
                        tempColour.y = chan0;
                        tempColour.cb = chan1;
                        tempColour.cr = chan2;
                        break;
                    case HSV:
                        tempColour.h = chan0;
                        tempColour.s = chan1;
                        tempColour.v = chan2;
                        break;
                    case RGB:
                        tempColour.r = chan0;
                        tempColour.g = chan1;
                        tempColour.b = chan2;
                        break;
                }
                result.push_back(convertToColourSpace(tempColour,currentSpace,desiredColourSpace));
            }
        }
    }
    return result;
}

ClassIndex::Colour ClassificationWidget::getColourLabel()
{
    return ClassIndex::getColourFromIndex(coloursComboBox->currentIndex());
}

// Slots
void ClassificationWidget::updateColourSpace(int newColourSpaceIndex)
{
    pixels::Pixel selectedColour;
    switch(newColourSpaceIndex)
    {
        case YCbCr:
            channelLabels[0]->setText(tr("Y: "));
            channelLabels[1]->setText(tr("Cb: "));
            channelLabels[2]->setText(tr("Cr: "));
            selectedColour = getCurrentColourInColourModel(YCbCr);
            currentColourSpace = YCbCr;
            break;
        case HSV:
            channelLabels[0]->setText(tr("Hue: "));
            channelLabels[1]->setText(tr("Saturation: "));
            channelLabels[2]->setText(tr("Value: "));
            // Convert the selected value to the new colour model if required.
            selectedColour = getCurrentColourInColourModel(HSV);
            currentColourSpace = HSV;
            break;
        case RGB:
            channelLabels[0]->setText(tr("Red: "));
            channelLabels[1]->setText(tr("Green: "));
            channelLabels[2]->setText(tr("Blue: "));
            // Convert the selected value to the new colour model if required.
            selectedColour = getCurrentColourInColourModel(RGB);
            currentColourSpace = RGB;
            break;
        default:
            break;
    };
    setColour(selectedColour, currentColourSpace);
    selectionChanged();
}

void ClassificationWidget::drawSelectedColour()
{
    pixels::Pixel drawingColour = getCurrentColourInColourModel(RGB);
    QPixmap temp_pixmap(22,22);
    temp_pixmap.fill(QColor(drawingColour.r, drawingColour.g, drawingColour.b));
    colourPreviewLabel->setPixmap(temp_pixmap);
}

void ClassificationWidget::setColour(pixels::Pixel colour, int colourSpace)
{
    ColourSpace currentSpace = getCurrentColourSpace();
    // Disable the signals that update the colour selection, otherwise weird things happen.
    for(int channel = 0; channel < numChannels; channel++)
    {
        channelSelectors[channel]->blockSignals(true);
    }

    // Convert if we need to.
    if(colourSpace != currentSpace){
        colour = convertToColourSpace(colour, colourSpace, currentSpace);
    }

    switch(getCurrentColourSpace()){
        case YCbCr:
            channelSelectors[0]->setValue(colour.y);
            channelSelectors[1]->setValue(colour.cb);
            channelSelectors[2]->setValue(colour.cr);
            break;
        case HSV:
            channelSelectors[0]->setValue(colour.h);
            channelSelectors[1]->setValue(colour.s);
            channelSelectors[2]->setValue(colour.v);
            break;
        case RGB:
            channelSelectors[0]->setValue(colour.r);
            channelSelectors[1]->setValue(colour.g);
            channelSelectors[2]->setValue(colour.b);
            break;
    }

    // Enable the signals that update the colour selection
    for(int channel = 0; channel < numChannels; channel++)
    {
        channelSelectors[channel]->blockSignals(false);
    }
    drawSelectedColour(); // Now force a redraw of the newly selected colour.
    selectionChanged(); // Signal that the colour selection has changed.
}

void ClassificationWidget::setAllBoundaries( int newBound)
{
    for(int channel = 0; channel < numChannels; channel++)
    {
        channelMinSelectors[channel]->setValue(-newBound);
        channelMaxSelectors[channel]->setValue(newBound);
    }
}
