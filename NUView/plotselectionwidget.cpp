#include "plotselectionwidget.h"

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
#include <qwt/qwt_symbol.h>
#include <qwt/qwt_plot_curve.h>

#include <typeinfo>
#include "GLDisplay.h"

PlotSelectionWidget::PlotSelectionWidget(QMdiArea* parentMdiWidget, QWidget *parent): QWidget(parent), mdiWidget(parentMdiWidget)
{
    setObjectName(tr("Plot Selection"));
    setWindowTitle(tr("Plot Selection"));

    createWidgets();
    createLayout();
    createConnections();
    this->setEnabled(false); // Dafault to disabled, until a valid window is selected.
    currentDisplay = 0;
    disableWriting = false;
}

PlotSelectionWidget::~PlotSelectionWidget()
{
    delete curveSelectionlayout;
    delete symbolSelectionLayout;
    delete styleSelectionLayout;
    delete overallLayout;

    delete curveComboBox;
    delete curveEnabledCheckBox;

    delete symbolLabel;
    delete styleLabel;

    delete symbolCombo;
    delete styleCombo;
}

void PlotSelectionWidget::createWidgets()
{
    curveComboBox = new QComboBox();

    map<QString, QwtPlotCurve*>::const_iterator cit;
    for(cit = PlotDisplay::curveMap.begin(); cit != PlotDisplay::curveMap.end(); cit++)
    {
        curveComboBox->addItem(cit->first);
    }

    // Checkboxes
    curveEnabledCheckBox = new QCheckBox("Enabled");

    symbolLabel = new QLabel("Symbol");
    symbolCombo = new QComboBox();

    for(int i=0; i<15; i++) {
        symbolCombo->addItem(getSymbolName(getSymbolFromInt(i)));
    }

    styleLabel = new QLabel("Style");
    styleCombo = new QComboBox();

    for(int i=0; i<5; i++) {
        styleCombo->addItem(getStyleName(getStyleFromInt(i)));
    }
}

void PlotSelectionWidget::createLayout()
{
    // Setup layer toggle controls
    curveSelectionlayout = new QHBoxLayout();
    curveSelectionlayout->addWidget(curveEnabledCheckBox,0,Qt::AlignHCenter);

    // Setup symbol selection row
    symbolSelectionLayout = new QHBoxLayout();
    symbolSelectionLayout->addWidget(symbolLabel,0,Qt::AlignHCenter);
    symbolSelectionLayout->addWidget(symbolColourButton,0,Qt::AlignHCenter);
    symbolSelectionLayout->addStretch(1);

    // Setup style selection row
    styleSelectionLayout = new QHBoxLayout();
    styleSelectionLayout->addWidget(styleLabel,0,Qt::AlignHCenter);
    styleSelectionLayout->addWidget(styleColourButton,0,Qt::AlignHCenter);
    styleSelectionLayout->addStretch(1);

    // Setup overall layout
    overallLayout = new QVBoxLayout();
    overallLayout->addWidget(curveComboBox);
    overallLayout->addLayout(curveSelectionlayout);
    overallLayout->addLayout(symbolSelectionLayout);
    overallLayout->addLayout(styleSelectionLayout);
    setLayout(overallLayout);
}

void PlotSelectionWidget::createConnections()
{
    // Setup signal to trigger a reload of current settings when a layer is selected.
    connect(curveComboBox,SIGNAL(activated(int)),this,SLOT(updateSelectedLayerSettings()));

    // Setup signal to notify when the currently selected window has changed.
    connect(mdiWidget,SIGNAL(subWindowActivated(QMdiSubWindow*)),this,SLOT(focusWindowChanged(QMdiSubWindow*)));

    // Signals for layer primary and enabled selections
    connect(curveEnabledCheckBox,SIGNAL(toggled(bool)),this,SLOT(enabledSettingChanged(bool)));

    // Signals to disable/enable elements when enabled layer is toggled.
    connect(curveEnabledCheckBox,SIGNAL(toggled(bool)),symbolLabel,SLOT(setEnabled(bool)));
    connect(curveEnabledCheckBox,SIGNAL(toggled(bool)),symbolCombo,SLOT(setEnabled(bool)));
    connect(curveEnabledCheckBox,SIGNAL(toggled(bool)),symbolColourButton,SLOT(setEnabled(bool)));
    connect(curveEnabledCheckBox,SIGNAL(toggled(bool)),styleLabel,SLOT(setEnabled(bool)));
    connect(curveEnabledCheckBox,SIGNAL(toggled(bool)),styleCombo,SLOT(setEnabled(bool)));
    connect(curveEnabledCheckBox,SIGNAL(toggled(bool)),styleColourButton,SLOT(setEnabled(bool)));

    // Setup Alpha selection signals
    connect(symbolCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(symbolSettingsChanged()));
    connect(symbolColourButton, SIGNAL(clicked()), this, SLOT(selectSymbolColourClicked()));
    connect(styleCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(styleSettingsChanged()));
    connect(styleColourButton, SIGNAL(clicked()), this, SLOT(selectStyleColourClicked()));
}


void PlotSelectionWidget::setSymbolColour(const QColor &newColour)
{
    if(newColour.isValid())
    {
        selectedSymbolColour = newColour;
        symbolSettingsChanged();
    }
}

void PlotSelectionWidget::setSymbolColour(const QString& newColourName)
{
    setSymbolColour(QColor(newColourName));
}

void PlotSelectionWidget::setStyleColour(const QColor &newColour)
{
    if(newColour.isValid())
    {
        selectedStyleColour = newColour;
        styleSettingsChanged();
    }
}

void PlotSelectionWidget::setStyleColour(const QString& newColourName)
{
    setStyleColour(QColor(newColourName));
}

void PlotSelectionWidget::selectSymbolColourClicked()
{
    QColor newColour = QColorDialog::getColor(getSelectedSymbolColour(),this);
    setSymbolColour(newColour);
}

void PlotSelectionWidget::selectStyleColourClicked()
{
    QColor newColour = QColorDialog::getColor(getSelectedStyleColour(),this);
    setStyleColour(newColour);
}

QColor PlotSelectionWidget::getSelectedSymbolColour()
{
    return selectedSymbolColour;
}

QColor PlotSelectionWidget::getSelectedStyleColour()
{
    return selectedStyleColour;
}

QString PlotSelectionWidget::getSelectedCurve()
{
    return curveComboBox->itemText(curveComboBox->currentIndex());
}

void PlotSelectionWidget::setSelectedCurve(QString newCurveName)
{
    curveComboBox->setCurrentIndex(curveComboBox->findText(newCurveName));
}

int PlotSelectionWidget::getDisplayHistoryIndex(PlotDisplay* display)
{
    int historyListIndex = -1;
    for (int listIndex = 0; listIndex < selectedCurveHistory.size(); listIndex++)
    {
        if(selectedCurveHistory.at(listIndex).first == display)
            historyListIndex = listIndex;
    }
    return historyListIndex;
 }

void PlotSelectionWidget::focusWindowChanged(QMdiSubWindow* focusWindow)
{
    if(focusWindow == NULL)
        return;

    QWidget* widget = focusWindow->widget();
    bool correctWindowType = (typeid(*widget) == typeid(PlotDisplay));

    // Enable if the selected window is the right type, disable otherwise.
    this->setEnabled(correctWindowType);

    // Save the currently selected layer.
    int historyListIndex = getDisplayHistoryIndex(currentDisplay);
    if(historyListIndex != -1)
    {
        selectedCurveHistory[historyListIndex].second = getSelectedCurve();
    }
    else
    {
        selectedCurveHistory.push_back(QPair<PlotDisplay*,QString>(currentDisplay,getSelectedCurve()));
    }

    if(correctWindowType)
    {
        // Assign to the current display
        currentDisplay = (PlotDisplay*)widget;

        // If available load the previously selected layer.
        historyListIndex = getDisplayHistoryIndex(currentDisplay);
        if(historyListIndex != -1)
        {
            setSelectedCurve(selectedCurveHistory[historyListIndex].second);
        }

        // Get the current settings for the selected layers from the new window.
        updateSelectedCurveSettings();
    }
    else
    {
        currentDisplay = NULL;
    }
}

void PlotSelectionWidget::updateSelectedCurveSettings()
{
    if(!currentDisplay) return;
    disableWriting = true;

    QString selectedCurveName = getSelectedCurve();

    map<QString, QwtPlotCurve*>::const_iterator curve_it = currentDisplay->curveMap.find(selectedCurveName);
    map<QString, bool>::const_iterator en_it = currentDisplay->curvesEnabled.find(selectedCurveName);
    if(curve_it == currentDisplay->curveMap.end() || en_it == currentDisplay->curvesEnabled.end())
        return;

    const QwtPlotCurve* selectedCurve = curve_it->second;

    // Update controls with stored values.
    curveEnabledCheckBox->setChecked(en_it->second);


    symbolCombo->setCurrentIndex(symbolCombo->findData(selectedCurve->symbol()->style()));
    if(selectedCurve->style() != NULL)
        styleCombo->setCurrentIndex(styleCombo->findData(selectedCurve->style()));
    setSymbolColour(selectedCurve->symbol()->pen().color());
    setStyleColour(selectedCurve->pen().color());

    disableWriting = false;
}

void PlotSelectionWidget::symbolSettingsChanged()
{
    if(!currentDisplay)
        return;

    if(disableWriting) return;

    QString selectedCurveName = getSelectedCurve();

    map<QString, QwtPlotCurve*>::const_iterator curve_it = currentDisplay->curveMap.find(selectedCurveName);

    if(curve_it == currentDisplay->curveMap.end())
        return;

    QPixmap tempPixmap(22,22);
    QPainter painter(&tempPixmap);

    QColor colour = getSelectedSymbolColour();
    QwtSymbol::Style style = getSymbolFromInt(symbolCombo->currentIndex());

    tempPixmap.fill(colour);
    painter.drawRect(0,0,tempPixmap.width(),tempPixmap.height());
    QIcon ButtonIcon(tempPixmap);
    symbolColourButton->setIcon(ButtonIcon);
    symbolColourButton->setIconSize(tempPixmap.rect().size());

    //can add alpha later if required
    //colour.setAlpha(alphaSlider->value());

    if(curveEnabledCheckBox->isChecked())
    {
        //assumes curve is already attached to plot (should be)
        QwtSymbol* symbol = new QwtSymbol(style);
        symbol->setColor(colour);
        curve_it->second->setSymbol(symbol);
        currentDisplay->replot();
    }
}

void PlotSelectionWidget::styleSettingsChanged()
{
    if(!currentDisplay)
        return;

    if(disableWriting) return;

    QString selectedCurveName = getSelectedCurve();

    map<QString, QwtPlotCurve*>::const_iterator curve_it = currentDisplay->curveMap.find(selectedCurveName);

    if(curve_it == currentDisplay->curveMap.end())
        return;

    QPixmap tempPixmap(22,22);
    QPainter painter(&tempPixmap);

    QColor colour = getSelectedStyleColour();
    QwtPlotCurve::CurveStyle style = getStyleFromInt(styleCombo->currentIndex());

    tempPixmap.fill(colour);
    painter.drawRect(0,0,tempPixmap.width(),tempPixmap.height());
    QIcon ButtonIcon(tempPixmap);
    styleColourButton->setIcon(ButtonIcon);
    styleColourButton->setIconSize(tempPixmap.rect().size());

    //can add alpha later if required
    //colour.setAlpha(alphaSlider->value());

    if(curveEnabledCheckBox->isChecked())
    {
        //assumes curve is already attached to plot (should be)
        curve_it->second->setPen(colour);
        curve_it->second->setStyle(style);
        currentDisplay->replot();
    }
}

void PlotSelectionWidget::enabledSettingChanged(bool enabled)
{
    if(disableWriting || !currentDisplay)
        return;

    QString selectedCurveName = getSelectedCurve();

    map<QString, QwtPlotCurve*>::const_iterator curve_it = currentDisplay->curveMap.find(selectedCurveName);

    if(curve_it == currentDisplay->curveMap.end())
        return;

    QPixmap tempPixmap(22,22);
    QPainter painter(&tempPixmap);

    QColor styleColour = getSelectedStyleColour();
    QColor symbolColour = getSelectedSymbolColour();
    QwtPlotCurve::CurveStyle style = getStyleFromInt(styleCombo->currentIndex());
    QwtSymbol::Style symbolStyle = getSymbolFromInt(symbolCombo->currentIndex());

    //can add alpha later if required
    //colour.setAlpha(alphaSlider->value());

    //assumes curve is already attached to plot (should be)
    QwtSymbol* symbol = new QwtSymbol(symbolStyle);
    symbol->setColor(symbolColour);
    curve_it->second->setSymbol(symbol);
    curve_it->second->setPen(styleColour);
    curve_it->second->setStyle(style);

    if(enabled)
        curve_it->second->attach(currentDisplay);
    else
        curve_it->second->detach();

    currentDisplay->replot();
}

void PlotSelectionWidget::primarySettingChanged(bool primary)
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


QwtSymbol::Style PlotSelectionWidget::getSymbolFromInt(int i)
{
    switch(i) {
    case 1: return QwtSymbol::Ellipse;
    case 2: return QwtSymbol::Rect;
    case 3: return QwtSymbol::Diamond;
    case 4: return QwtSymbol::UTriangle;
    case 5: return QwtSymbol::DTriangle;
    case 6: return QwtSymbol::LTriangle;
    case 7: return QwtSymbol::RTriangle;
    case 8: return QwtSymbol::Cross;
    case 9: return QwtSymbol::XCross;
    case 10: return QwtSymbol::HLine;
    case 11: return QwtSymbol::VLine;
    case 12: return QwtSymbol::Star1;
    case 13: return QwtSymbol::Star2;
    case 14: return QwtSymbol::Hexagon;
    default: return QwtSymbol::NoSymbol;
    }
}

QwtPlotCurve::CurveStyle PlotSelectionWidget::getStyleFromInt(int i)
{
    switch(i) {
    case 1: return QwtPlotCurve::Lines;
    case 2: return QwtPlotCurve::Sticks;
    case 3: return QwtPlotCurve::Steps;
    case 4: return QwtPlotCurve::Dots;
    default: return QwtPlotCurve::NoCurve;
    }
}

static QString PlotSelectionWidget::getSymbolName(QwtSymbol::Style id)
{
    switch(id) {
    case QwtSymbol::NoSymbol: return "No Symbol";
    case QwtSymbol::Ellipse: return "Ellipse";
    case QwtSymbol::Rect: return "Rectangle";
    case QwtSymbol::Diamond: return "Diamond";
    case QwtSymbol::UTriangle: return "Triangle (up)";
    case QwtSymbol::DTriangle: return "Triangle (down)";
    case QwtSymbol::LTriangle: return "Triangle (left)";
    case QwtSymbol::RTriangle: return "Triangle (right)";
    case QwtSymbol::Cross: return "Cross (+)";
    case QwtSymbol::XCross: return "Cross (X)";
    case QwtSymbol::HLine: return "Horizontal Bar";
    case QwtSymbol::VLine: return "Vertical Bar";
    case QwtSymbol::Star1: return "Asterisk";
    case QwtSymbol::Star2: return "Star";
    case QwtSymbol::Hexagon: return "Hexagon";
    default: return "Invalid";
    }
}

static QString PlotSelectionWidget::getStyleName(QwtPlotCurve::CurveStyle id)
{
    switch(id) {
    case QwtPlotCurve::NoCurve: return "No Curve";
    case QwtPlotCurve::Lines: return "Lines";
    case QwtPlotCurve::Sticks: return "Sticks";
    case QwtPlotCurve::Steps: return "Steps";
    case QwtPlotCurve::Dots: return "Dots";
    default: return "Invalid";
    }
}
