#include "LayerSelectionWidget.h"
#include <QMdiArea>
#include <QComboBox>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QMdiSubWindow>
#include <typeinfo>
#include <QDebug>
#include "GLDisplay.h"

LayerSelectionWidget::LayerSelectionWidget(QMdiArea* parentMdiWidget, QWidget *parent): QWidget(parent), mdiWidget(parentMdiWidget)
{
    setObjectName(tr("Layer Selection"));
    setWindowTitle(tr("Layer Selection"));
    createWidgets();
    createLayout();
    createConnections();
    layerEnabledCheckBox->setChecked(true);
    layerEnabledCheckBox->toggle();
    currentDisplay = 0;
    disableWriting = false;
}

LayerSelectionWidget::~LayerSelectionWidget()
{
    delete layerSelectionlayout;
    delete colourSelectionLayout;
    delete overallLayout;

    delete layerComboBox;

    delete layerPrimaryCheckBox;
    delete layerEnabledCheckBox;

    delete redLabel;
    delete greenLabel;
    delete blueLabel;
    delete alphaLabel;

    delete redSlider;
    delete greenSlider;
    delete blueSlider;
    delete alphaSlider;

    delete redSpinBox;
    delete greenSpinBox;
    delete blueSpinBox;
    delete alphaSpinBox;
}

void LayerSelectionWidget::createWidgets()
{
    layerComboBox = new QComboBox();
    for(int id = 1; id < GLDisplay::numDisplays; id++)
    {
        layerComboBox->addItem(GLDisplay::getLayerName(id),id);
    }

    layerPrimaryCheckBox = new QCheckBox("Primary");
    layerEnabledCheckBox = new QCheckBox("Enabled");

    redLabel = new QLabel("Red");
    greenLabel = new QLabel("Green");
    blueLabel = new QLabel("Blue");
    alphaLabel = new QLabel("Alpha");

    redSlider = new QSlider(Qt::Horizontal);
    redSlider->setMinimum(0);
    redSlider->setMaximum(255);

    greenSlider = new QSlider(Qt::Horizontal);
    greenSlider->setMinimum(0);
    greenSlider->setMaximum(255);

    blueSlider = new QSlider(Qt::Horizontal);
    blueSlider->setMinimum(0);
    blueSlider->setMaximum(255);

    alphaSlider = new QSlider(Qt::Horizontal);
    alphaSlider->setMinimum(0);
    alphaSlider->setMaximum(255);

    redSpinBox = new QSpinBox();
    redSpinBox->setMinimum(redSlider->minimum());
    redSpinBox->setMaximum(redSlider->maximum());

    greenSpinBox = new QSpinBox();
    greenSpinBox->setMinimum(greenSlider->minimum());
    greenSpinBox->setMaximum(greenSlider->maximum());

    blueSpinBox = new QSpinBox();
    blueSpinBox->setMinimum(blueSlider->minimum());
    blueSpinBox->setMaximum(blueSlider->maximum());

    alphaSpinBox = new QSpinBox();
    alphaSpinBox->setMinimum(alphaSlider->minimum());
    alphaSpinBox->setMaximum(alphaSlider->maximum());
}

void LayerSelectionWidget::createLayout()
{

    layerSelectionlayout = new QHBoxLayout();
    //layerSelectionlayout->addWidget(layerComboBox,1);
    layerSelectionlayout->addWidget(layerPrimaryCheckBox,0,Qt::AlignHCenter);
    layerSelectionlayout->addWidget(layerEnabledCheckBox,0,Qt::AlignHCenter);

    colourSelectionLayout = new QGridLayout();
    colourSelectionLayout->addWidget(redLabel,0,0);
    colourSelectionLayout->addWidget(redSlider,0,1);
    colourSelectionLayout->addWidget(redSpinBox,0,2);

    colourSelectionLayout->addWidget(greenLabel,1,0);
    colourSelectionLayout->addWidget(greenSlider,1,1);
    colourSelectionLayout->addWidget(greenSpinBox,1,2);

    colourSelectionLayout->addWidget(blueLabel,2,0);
    colourSelectionLayout->addWidget(blueSlider,2,1);
    colourSelectionLayout->addWidget(blueSpinBox,2,2);

    colourSelectionLayout->addWidget(alphaLabel,3,0);
    colourSelectionLayout->addWidget(alphaSlider,3,1);
    colourSelectionLayout->addWidget(alphaSpinBox,3,2);

    colourSelectionLayout->setColumnStretch(1,1);

    overallLayout = new QVBoxLayout();
    overallLayout->addWidget(layerComboBox);
    overallLayout->addLayout(layerSelectionlayout);
    overallLayout->addLayout(colourSelectionLayout);
    setLayout(overallLayout);
}

void LayerSelectionWidget::createConnections()
{
    connect(layerComboBox,SIGNAL(activated(int)),this,SLOT(updateSelectedLayerSettings()));
    connect(mdiWidget,SIGNAL(subWindowActivated(QMdiSubWindow*)),this,SLOT(focusWindowChanged(QMdiSubWindow*)));

    connect(layerPrimaryCheckBox,SIGNAL(toggled(bool)),layerEnabledCheckBox,SLOT(setDisabled(bool)));
    connect(layerPrimaryCheckBox,SIGNAL(toggled(bool)),layerPrimaryCheckBox,SLOT(setDisabled(bool)));

    connect(layerPrimaryCheckBox,SIGNAL(toggled(bool)),this,SLOT(primarySettingChanged(bool)));
    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),this,SLOT(enabledSettingChanged(bool)));


    connect(layerPrimaryCheckBox,SIGNAL(toggled(bool)),this,SLOT(isPrimaryChanged(bool)));


    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),redSlider,SLOT(setEnabled(bool)));
    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),redSpinBox,SLOT(setEnabled(bool)));

    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),blueSlider,SLOT(setEnabled(bool)));
    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),blueSpinBox,SLOT(setEnabled(bool)));

    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),greenSlider,SLOT(setEnabled(bool)));
    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),greenSpinBox,SLOT(setEnabled(bool)));

    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),alphaSlider,SLOT(setEnabled(bool)));
    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),alphaSpinBox,SLOT(setEnabled(bool)));

    // Red
    connect(redSlider,SIGNAL(valueChanged(int)),redSpinBox,SLOT(setValue(int)));
    connect(redSpinBox,SIGNAL(valueChanged(int)),redSlider,SLOT(setValue(int)));
    // Green
    connect(greenSlider,SIGNAL(valueChanged(int)),greenSpinBox,SLOT(setValue(int)));
    connect(greenSpinBox,SIGNAL(valueChanged(int)),greenSlider,SLOT(setValue(int)));
    // Blue
    connect(blueSlider,SIGNAL(valueChanged(int)),blueSpinBox,SLOT(setValue(int)));
    connect(blueSpinBox,SIGNAL(valueChanged(int)),blueSlider,SLOT(setValue(int)));
    // Alpha
    connect(alphaSlider,SIGNAL(valueChanged(int)),alphaSpinBox,SLOT(setValue(int)));
    connect(alphaSpinBox,SIGNAL(valueChanged(int)),alphaSlider,SLOT(setValue(int)));

    connect(redSlider,SIGNAL(valueChanged(int)),this,SLOT(colourSettingsChanged()));
    connect(greenSlider,SIGNAL(valueChanged(int)),this,SLOT(colourSettingsChanged()));
    connect(blueSlider,SIGNAL(valueChanged(int)),this,SLOT(colourSettingsChanged()));
    connect(alphaSlider,SIGNAL(valueChanged(int)),this,SLOT(colourSettingsChanged()));

}


void LayerSelectionWidget::focusWindowChanged(QMdiSubWindow* focusWindow)
{
    if(focusWindow == 0) return;
    QWidget* widget = focusWindow->widget();
    bool correctWindowType = (typeid(*widget) == typeid(GLDisplay));

    // Enable if valid window type, disable if not.
    this->setEnabled(correctWindowType);

    if(correctWindowType)
    {
        currentDisplay = (GLDisplay*)widget;
        updateSelectedLayerSettings();
    }
    else
    {
        currentDisplay = 0;
    }
}


void LayerSelectionWidget::isPrimaryChanged(bool isPrimary)
{
    if(isPrimary)
    {
        layerEnabledCheckBox->setChecked(true);
    }
}

void LayerSelectionWidget::updateSelectedLayerSettings()
{
    if(!currentDisplay) return;
    disableWriting = true;

    int selectedIndex = layerComboBox->currentIndex();
    int selectedLayerID = layerComboBox->itemData(selectedIndex).toInt();

    const GLDisplay::Layer* selectedLayer = currentDisplay->getLayerSettings(selectedLayerID);

    // Update controls with stored values.
    layerPrimaryCheckBox->setChecked(selectedLayer->primary);
    layerEnabledCheckBox->setChecked(selectedLayer->enabled);
    redSlider->setValue(selectedLayer->colour.red());
    greenSlider->setValue(selectedLayer->colour.green());
    blueSlider->setValue(selectedLayer->colour.blue());
    alphaSlider->setValue(selectedLayer->colour.alpha());
    disableWriting = false;
}

void LayerSelectionWidget::colourSettingsChanged()
{
    if(disableWriting || !currentDisplay) return;
    int selectedIndex = layerComboBox->currentIndex();
    int selectedLayerID = layerComboBox->itemData(selectedIndex).toInt();

    QColor colour(redSlider->value(), greenSlider->value(), blueSlider->value(), alphaSlider->value());
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
    int selectedIndex = layerComboBox->currentIndex();
    int selectedLayerID = layerComboBox->itemData(selectedIndex).toInt();

    QColor colour(redSlider->value(), greenSlider->value(), blueSlider->value(), alphaSlider->value());
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
    if(disableWriting || !currentDisplay) return;
    
    int selectedIndex = layerComboBox->currentIndex();
    int selectedLayerID = layerComboBox->itemData(selectedIndex).toInt();

    QColor colour(redSlider->value(), greenSlider->value(), blueSlider->value(), alphaSlider->value());
    if(layerPrimaryCheckBox->isChecked())
    {
        currentDisplay->setPrimaryDisplay(selectedLayerID);
        currentDisplay->update();
    }
}
