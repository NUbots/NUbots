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
#include <QStringlist>
#include <QPixmap>
#include "GLDisplay.h"

LayerSelectionWidget::LayerSelectionWidget(QMdiArea* parentMdiWidget, QWidget *parent): QWidget(parent), mdiWidget(parentMdiWidget)
{
    setObjectName(tr("Layer Selection"));
    setWindowTitle(tr("Layer Selection"));

    coloursList << "red" << "green" << "blue" << "yellow" << "orange" << "black" << "white" << "royalblue" << "purple" << "pink" << "limegreen" << "peru" << "hotpink";


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
    delete overallLayout;

    delete layerComboBox;

    delete layerPrimaryCheckBox;
    delete layerEnabledCheckBox;

    delete drawingColourLabel;
    delete colourLabel;
    delete alphaLabel;

    delete colourSlider;
    delete alphaSlider;

    delete colourSpinBox;
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

    QPixmap tempPixmap(22,22);
    tempPixmap.fill(QColor(Qt::white));
    drawingColourLabel = new QLabel();
    drawingColourLabel->setPixmap(tempPixmap);
    colourLabel = new QLabel("Colour");
    alphaLabel = new QLabel("Alpha");

    colourSlider = new QSlider(Qt::Horizontal);
    colourSlider->setMinimum(0);
    colourSlider->setMaximum(coloursList.length()-1);

    alphaSlider = new QSlider(Qt::Horizontal);
    alphaSlider->setMinimum(0);
    alphaSlider->setMaximum(255);

    colourSpinBox = new QSpinBox();
    colourSpinBox->setMinimum(colourSlider->minimum());
    colourSpinBox->setMaximum(colourSlider->maximum());

    alphaSpinBox = new QSpinBox();
    alphaSpinBox->setMinimum(alphaSlider->minimum());
    alphaSpinBox->setMaximum(alphaSlider->maximum());
}

void LayerSelectionWidget::createLayout()
{

    layerSelectionlayout = new QHBoxLayout();
    layerSelectionlayout->addWidget(layerPrimaryCheckBox,0,Qt::AlignHCenter);
    layerSelectionlayout->addWidget(layerEnabledCheckBox,0,Qt::AlignHCenter);

    colourSelectionLayout = new QGridLayout();
    colourSelectionLayout->addWidget(drawingColourLabel,0,0);
    colourSelectionLayout->addWidget(colourLabel,0,1);
    colourSelectionLayout->addWidget(colourSlider,0,2);
    colourSelectionLayout->addWidget(colourSpinBox,0,3);

    colourSelectionLayout->addWidget(alphaLabel,1,1);
    colourSelectionLayout->addWidget(alphaSlider,1,2);
    colourSelectionLayout->addWidget(alphaSpinBox,1,3);

    colourSelectionLayout->setColumnStretch(2,1);

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

    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),colourSlider,SLOT(setEnabled(bool)));
    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),colourSpinBox,SLOT(setEnabled(bool)));
    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),colourLabel,SLOT(setEnabled(bool)));

    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),alphaSlider,SLOT(setEnabled(bool)));
    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),alphaSpinBox,SLOT(setEnabled(bool)));
    connect(layerEnabledCheckBox,SIGNAL(toggled(bool)),alphaLabel,SLOT(setEnabled(bool)));

    // Red
    connect(colourSlider,SIGNAL(valueChanged(int)),colourSpinBox,SLOT(setValue(int)));
    connect(colourSpinBox,SIGNAL(valueChanged(int)),colourSlider,SLOT(setValue(int)));

    // Alpha
    connect(alphaSlider,SIGNAL(valueChanged(int)),alphaSpinBox,SLOT(setValue(int)));
    connect(alphaSpinBox,SIGNAL(valueChanged(int)),alphaSlider,SLOT(setValue(int)));

    connect(colourSlider,SIGNAL(valueChanged(int)),this,SLOT(colourSettingsChanged()));
    connect(alphaSlider,SIGNAL(valueChanged(int)),this,SLOT(colourSettingsChanged()));
}

QColor LayerSelectionWidget::getSelectedColour()
{
    return QColor(coloursList[colourSlider->value()]);
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

    QString closestColourName = getClosestColourName(selectedLayer->colour);
    int index = coloursList.indexOf(closestColourName);
    colourSlider->setValue(index);
    //redSlider->setValue(selectedLayer->colour.red());
    alphaSlider->setValue(selectedLayer->colour.alpha());
    disableWriting = false;
}

void LayerSelectionWidget::colourSettingsChanged()
{
    if(disableWriting || !currentDisplay) return;
    int selectedIndex = layerComboBox->currentIndex();
    int selectedLayerID = layerComboBox->itemData(selectedIndex).toInt();

    QColor colour = getSelectedColour();
    colour.setAlpha(alphaSlider->value());

    QPixmap tempPixmap(22,22);
    tempPixmap.fill(colour);
    drawingColourLabel->setPixmap(tempPixmap);

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
    if(disableWriting || !currentDisplay) return;
    
    int selectedIndex = layerComboBox->currentIndex();
    int selectedLayerID = layerComboBox->itemData(selectedIndex).toInt();

    QColor colour = getSelectedColour();
    colour.setAlpha(alphaSlider->value());
    if(layerPrimaryCheckBox->isChecked())
    {
        currentDisplay->setPrimaryDisplay(selectedLayerID);
        currentDisplay->update();
    }
}

QString LayerSelectionWidget::getClosestColourName(QColor colour)
{
    QColor tempCol;
    QString closestColourName;
    int closestDistance = 255*255*255;
    int tempDistance;
    for (int i = 0; i < coloursList.length(); i++)
    {
        tempCol.setNamedColor(coloursList[i]);
        tempDistance = abs(tempCol.red() - colour.red());
        tempDistance += abs(tempCol.green() - colour.green());
        tempDistance += abs(tempCol.blue() - colour.blue());
        if(tempDistance < closestDistance)
        {
            closestDistance = tempDistance;
            closestColourName = coloursList[i];
        }
    }
    return closestColourName;
}
