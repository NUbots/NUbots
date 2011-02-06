#include "LayerSelectionWidget.h"

// Qt Includes
#include <QMdiArea>
#include <QComboBox>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QMdiSubWindow>
#include <QPixmap>
#include <QPushButton>
#include <QSignalMapper>
#include <QPainter>
#include <QToolButton>
#include <QColorDialog>

#include <typeinfo>
#include "GLDisplay.h"

LayerSelectionWidget::LayerSelectionWidget(QMdiArea* parentMdiWidget, QWidget *parent): QWidget(parent), mdiWidget(parentMdiWidget), quickSelectColoursPerLine(8)
{
    setObjectName(tr("Layer Selection"));
    setWindowTitle(tr("Layer Selection"));

    quickSelectColoursList << "white" << "yellow" << "orange" << "red" << "green" << "blue" << "black" << "purple" << "teal" << "hotpink" << "silver" << "khaki";

    createWidgets();
    createLayout();
    createConnections();
    this->setEnabled(false); // Dafault to disabled, until a valid window is selected.
    currentDisplay = 0;
    disableWriting = false;

}

LayerSelectionWidget::~LayerSelectionWidget()
{
    delete layerSelectionlayout;
    delete colourSelectionLayout;
    delete quickColourSelectionLayout;
    delete alphaSelectionlayout;
    delete overallLayout;

    delete layerComboBox;
    delete layerPrimaryCheckBox;
    delete layerEnabledCheckBox;

    delete colourLabel;
    delete drawingColourLabel;
    delete alphaLabel;

    delete alphaSlider;
    delete alphaSpinBox;

    for (int colId = 0; colId < quickSelectPushButtons.size(); colId++)
    {
        delete quickSelectPushButtons[colId];
    }
    delete quickSelectSignalMapping;
}

void LayerSelectionWidget::createWidgets()
{
    layerComboBox = new QComboBox();
    for(int id = 1; id < GLDisplay::numDisplays; id++)
    {
        layerComboBox->addItem(GLDisplay::getLayerName(id),id);
    }

    // Checkboxes
    layerPrimaryCheckBox = new QCheckBox("Primary");
    layerEnabledCheckBox = new QCheckBox("Enabled");

    // Pixmaps for icons
    QPixmap tempPixmap(22,22);
    QPixmap smallTempPixmap(15,15);

    // Make the selected colour labels
    tempPixmap.fill(QColor(Qt::transparent));
    drawingColourLabel = new QLabel();
    drawingColourLabel->setPixmap(tempPixmap);
    colourLabel = new QLabel("Drawing Colour");

    // Make the quick select colour buttons.
    quickSelectPushButtons.resize(quickSelectColoursList.size());
    QPainter painter(&smallTempPixmap);
    for (int colId = 0; colId < quickSelectColoursList.size(); colId++)
    {
        quickSelectPushButtons[colId] = new QToolButton();
        smallTempPixmap.fill(QColor(quickSelectColoursList[colId]));
        painter.drawRect(0,0,smallTempPixmap.width(),smallTempPixmap.height());
        quickSelectPushButtons[colId]->setIcon(QIcon(smallTempPixmap));
        quickSelectPushButtons[colId]->setAutoRaise(true);
    }

    alphaLabel = new QLabel("Alpha");
    alphaSlider = new QSlider(Qt::Horizontal);
    alphaSlider->setMinimum(0);
    alphaSlider->setMaximum(255);

    selectCustomColourPushButton = new QPushButton("Custom...");

    alphaSpinBox = new QSpinBox();
    alphaSpinBox->setMinimum(alphaSlider->minimum());
    alphaSpinBox->setMaximum(alphaSlider->maximum());
}

void LayerSelectionWidget::createLayout()
{

    // Setup layer toggle controls
    layerSelectionlayout = new QHBoxLayout();
    layerSelectionlayout->addWidget(layerPrimaryCheckBox,0,Qt::AlignHCenter);
    layerSelectionlayout->addWidget(layerEnabledCheckBox,0,Qt::AlignHCenter);

    // Setup quick colour selection buttons
    quickColourSelectionLayout = new QGridLayout();
    quickColourSelectionLayout->setSpacing(0);
    quickColourSelectionLayout->setMargin(0);
    int row, collumn;
    for (int colId = 0; colId < quickSelectPushButtons.size(); colId++)
    {
        row = colId / quickSelectColoursPerLine;
        collumn = colId % quickSelectColoursPerLine;
        quickColourSelectionLayout->addWidget(quickSelectPushButtons[colId],row,collumn);
    }

    // Setup colour selection row
    colourSelectionLayout = new QHBoxLayout();
    colourSelectionLayout->addWidget(colourLabel,0,Qt::AlignHCenter);
    colourSelectionLayout->addWidget(drawingColourLabel,0,Qt::AlignHCenter);
    colourSelectionLayout->addWidget(selectCustomColourPushButton,0,Qt::AlignHCenter);
    colourSelectionLayout->addStretch(1);

    // Setup alpha selection row
    alphaSelectionlayout = new QHBoxLayout();
    alphaSelectionlayout->addWidget(alphaLabel);
    alphaSelectionlayout->addWidget(alphaSlider);
    alphaSelectionlayout->addWidget(alphaSpinBox);

    // Setup overall layout
    overallLayout = new QVBoxLayout();
    overallLayout->addWidget(layerComboBox);
    overallLayout->addLayout(layerSelectionlayout);
    overallLayout->addLayout(quickColourSelectionLayout);
    overallLayout->addLayout(colourSelectionLayout);
    overallLayout->addLayout(alphaSelectionlayout);
    setLayout(overallLayout);
}

void LayerSelectionWidget::createConnections()
{
    // Setup signal to trigger a reload of current settings when a layer is selected.
    connect(layerComboBox,SIGNAL(activated(int)),this,SLOT(updateSelectedLayerSettings()));

    // Setup signal to notify when the currently selected window has changed.
    connect(mdiWidget,SIGNAL(subWindowActivated(QMdiSubWindow*)),this,SLOT(focusWindowChanged(QMdiSubWindow*)));

    // Signals for layer primary and enabled selections
    connect(layerPrimaryCheckBox,SIGNAL(toggled(bool)),this,SLOT(primarySettingChanged(bool)));
    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),this,SLOT(enabledSettingChanged(bool)));

    // Signals to disable/enable elements when primary layer is toggled.
    connect(layerPrimaryCheckBox,SIGNAL(toggled(bool)),layerEnabledCheckBox,SLOT(setDisabled(bool)));
    connect(layerPrimaryCheckBox,SIGNAL(toggled(bool)),layerPrimaryCheckBox,SLOT(setDisabled(bool)));

    // Signals to disable/enable elements when enabled layer is toggled.
    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),colourLabel,SLOT(setEnabled(bool)));
    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),drawingColourLabel,SLOT(setEnabled(bool)));
    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),alphaSlider,SLOT(setEnabled(bool)));
    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),alphaSpinBox,SLOT(setEnabled(bool)));
    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),alphaLabel,SLOT(setEnabled(bool)));
    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),selectCustomColourPushButton,SLOT(setEnabled(bool)));

    // Setup the quick select signal mappings.
    quickSelectSignalMapping = new QSignalMapper(this);
    for (int colId = 0; colId < quickSelectPushButtons.size(); colId++)
    {
        quickSelectSignalMapping->setMapping(quickSelectPushButtons[colId],quickSelectColoursList[colId]);
        connect(quickSelectPushButtons[colId], SIGNAL(clicked()),quickSelectSignalMapping, SLOT (map()));
        connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),quickSelectPushButtons[colId],SLOT(setEnabled(bool)));

    }
    connect(quickSelectSignalMapping, SIGNAL(mapped(const QString &)), this, SLOT(setColour(const QString &)));

    // Connect custom colour button to its function.
    connect(selectCustomColourPushButton, SIGNAL(clicked()),this, SLOT(selectCustomColourClicked()));

    // Setup Alpha selection signals
    connect(alphaSlider,SIGNAL(valueChanged(int)),alphaSpinBox,SLOT(setValue(int)));
    connect(alphaSpinBox,SIGNAL(valueChanged(int)),alphaSlider,SLOT(setValue(int)));
    connect(alphaSlider,SIGNAL(valueChanged(int)),this,SLOT(colourSettingsChanged()));
}


void LayerSelectionWidget::setColour(const QColor& newColour)
{
    if(newColour.isValid())
    {
        selectedColour = newColour;
        colourSettingsChanged();
    }
}

void LayerSelectionWidget::setColour(const QString& newColourName)
{
    setColour(QColor(newColourName));
}

void LayerSelectionWidget::selectCustomColourClicked()
{
    QColor newColour = QColorDialog::getColor(getSelectedColour(),this);
    setColour(newColour);
}

QColor LayerSelectionWidget::getSelectedColour()
{
    return selectedColour;
}

int LayerSelectionWidget::getSelectedLayer()
{
    int selectedIndex = layerComboBox->currentIndex();
    return layerComboBox->itemData(selectedIndex).toInt();
}

void LayerSelectionWidget::setSelectedLayer(int newLayerId)
{
    int selectedIndex = layerComboBox->findData(newLayerId);
    layerComboBox->setCurrentIndex(selectedIndex);
    return;
}

 int LayerSelectionWidget::getDisplayHistoryIndex(GLDisplay* display)
 {
    int historyListIndex = -1;
    for (int listIndex = 0; listIndex < selectedLayerHistory.size(); listIndex++)
    {
        if(selectedLayerHistory.at(listIndex).first == display)
            historyListIndex = listIndex;
    }
    return historyListIndex;
 }

void LayerSelectionWidget::focusWindowChanged(QMdiSubWindow* focusWindow)
{
    if(focusWindow == 0) return;
    QWidget* widget = focusWindow->widget();
    bool correctWindowType = (typeid(*widget) == typeid(GLDisplay));

    // Enable if the selected window is the right type, disable otherwise.
    this->setEnabled(correctWindowType);

    // Save the currently selected layer.
    int historyListIndex = getDisplayHistoryIndex(currentDisplay);
    if(historyListIndex != -1)
    {
        selectedLayerHistory[historyListIndex].second = getSelectedLayer();
    }
    else
    {
        selectedLayerHistory.push_back(QPair<GLDisplay*,int>(currentDisplay,getSelectedLayer()));
    }

    if(correctWindowType)
    {
        // Assign to the current display
        currentDisplay = (GLDisplay*)widget;

        // If available load the previously selected layer.
        historyListIndex = getDisplayHistoryIndex(currentDisplay);
        if(historyListIndex != -1)
        {
            setSelectedLayer(selectedLayerHistory[historyListIndex].second);
        }
        else
        {
            // Otherwise sset it to the primary layer if there is one.
            int primaryId = currentDisplay->getPrimaryLayerId();
            if(primaryId != GLDisplay::unknown)
            {
                setSelectedLayer(primaryId);
            }
        }

        // Get the current settings for the selected layers from the new window.
        updateSelectedLayerSettings();
    }
    else
    {
        currentDisplay = 0;
    }
}

void LayerSelectionWidget::updateSelectedLayerSettings()
{
    if(!currentDisplay) return;
    disableWriting = true;

    int selectedLayerID = getSelectedLayer();

    const GLDisplay::Layer* selectedLayer = currentDisplay->getLayerSettings(selectedLayerID);

    // Update controls with stored values.
    layerPrimaryCheckBox->setChecked(selectedLayer->primary);
    layerEnabledCheckBox->setChecked(selectedLayer->enabled);

    setColour(selectedLayer->colour);
    alphaSlider->setValue(selectedLayer->colour.alpha());
    disableWriting = false;
}

void LayerSelectionWidget::colourSettingsChanged()
{
    if(!currentDisplay) return;

    QPixmap tempPixmap(22,22);
    QPainter painter(&tempPixmap);

    QColor colour = getSelectedColour();
    tempPixmap.fill(colour);
    painter.drawRect(0,0,tempPixmap.width(),tempPixmap.height());
    drawingColourLabel->setPixmap(tempPixmap);

    if(disableWriting) return;
    int selectedLayerID = getSelectedLayer();
    colour.setAlpha(alphaSlider->value());

    if(layerPrimaryCheckBox->isChecked())
    {
        currentDisplay->setPrimaryDisplay(selectedLayerID,colour);
        currentDisplay->update();
    }
    else
    {
        currentDisplay->setOverlayDrawing(selectedLayerID, layerEnabledCheckBox->isChecked(),colour);
        currentDisplay->update();
    }
}

void LayerSelectionWidget::enabledSettingChanged(bool enabled)
{
    if(disableWriting || !currentDisplay) return;
    int selectedLayerID = getSelectedLayer();

    QColor colour = getSelectedColour();
    colour.setAlpha(alphaSlider->value());

    if(layerPrimaryCheckBox->isChecked())
    {
        return;
    }
    else
    {
        currentDisplay->setOverlayDrawing(selectedLayerID, layerEnabledCheckBox->isChecked());
        currentDisplay->update();
    }

}

void LayerSelectionWidget::primarySettingChanged(bool primary)
{
    if(!currentDisplay) return;

    if(primary)
    {
        layerEnabledCheckBox->setChecked(true);
    }

    if(disableWriting) return;
    
    int selectedLayerID = getSelectedLayer();

    QColor colour = getSelectedColour();
    colour.setAlpha(alphaSlider->value());
    if(layerPrimaryCheckBox->isChecked())
    {
        currentDisplay->setPrimaryDisplay(selectedLayerID);
        currentDisplay->update();
    }
}
