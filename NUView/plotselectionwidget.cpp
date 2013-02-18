/// @author Shannon Fenn

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

#include <boost/foreach.hpp>

#include <typeinfo>
#include "GLDisplay.h"

PlotSelectionWidget::PlotSelectionWidget(QMdiArea* parentMdiWidget, QWidget *parent):
    QWidget(parent),
    mdiWidget(parentMdiWidget),
    selectedSymbol(QwtSymbol::NoSymbol)
{
    newSymbolDialog = new CreateQwtSymbolDialog;
    newSymbolDialog->hide();
    setObjectName(tr("Plot Selection"));
    setWindowTitle(tr("Plot Selection"));

    createWidgets();
    createLayout();
    createConnections();
    this->setEnabled(false); // Dafault to disabled, until a valid window is selected.
    currentDisplay = 0;
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

    delete styleCombo;

    delete symbolButton;
    delete styleColourButton;

    delete newSymbolDialog;
}

void PlotSelectionWidget::createWidgets()
{
    curveComboBox = new QComboBox();

    // Checkboxes
    curveEnabledCheckBox = new QCheckBox("Enabled");

    symbolLabel = new QLabel("Symbol");
    symbolButton = new QToolButton();

    styleLabel = new QLabel("Style");
    styleCombo = new QComboBox();
    styleColourButton = new QToolButton();

    for(int i=0; i<5; i++) {
        styleCombo->addItem(getStyleName(getStyleFromInt(i)));
    }
}

void PlotSelectionWidget::createLayout()
{
    // Setup layer toggle controls
    curveSelectionlayout = new QHBoxLayout();
    curveSelectionlayout->addWidget(curveEnabledCheckBox,0,Qt::AlignHCenter);
    curveSelectionlayout->addStretch(1);

    // Setup symbol selection row
    symbolSelectionLayout = new QHBoxLayout();
    symbolSelectionLayout->addWidget(symbolLabel,0,Qt::AlignHCenter);
    symbolSelectionLayout->addWidget(symbolButton,0,Qt::AlignHCenter);
    symbolSelectionLayout->addStretch(1);

    // Setup style selection row
    styleSelectionLayout = new QHBoxLayout();
    styleSelectionLayout->addWidget(styleLabel,0,Qt::AlignHCenter);
    styleSelectionLayout->addWidget(styleCombo,0,Qt::AlignHCenter);
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
    connect(curveComboBox,SIGNAL(activated(int)),this,SLOT(updateSelectedCurveSettings()));

    // Setup signal to notify when the currently selected window has changed.
    connect(mdiWidget,SIGNAL(subWindowActivated(QMdiSubWindow*)),this,SLOT(focusWindowChanged(QMdiSubWindow*)));

    // Signals for layer primary and enabled selections
    connect(curveEnabledCheckBox,SIGNAL(toggled(bool)),this,SLOT(displaySettingsChanged()));

    // Signals to disable/enable elements when enabled layer is toggled.
    connect(curveEnabledCheckBox,SIGNAL(toggled(bool)),symbolLabel,SLOT(setEnabled(bool)));
    connect(curveEnabledCheckBox,SIGNAL(toggled(bool)),symbolButton,SLOT(setEnabled(bool)));
    connect(curveEnabledCheckBox,SIGNAL(toggled(bool)),styleLabel,SLOT(setEnabled(bool)));
    connect(curveEnabledCheckBox,SIGNAL(toggled(bool)),styleCombo,SLOT(setEnabled(bool)));
    connect(curveEnabledCheckBox,SIGNAL(toggled(bool)),styleColourButton,SLOT(setEnabled(bool)));

    // Setup symbol and style selection signals
    connect(symbolButton, SIGNAL(clicked()), this, SLOT(selectSymbolClicked()));
    connect(styleCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(displaySettingsChanged()));
    connect(styleColourButton, SIGNAL(clicked()), this, SLOT(selectStyleColourClicked()));

    connect(newSymbolDialog, SIGNAL(newSymbolCreated(QwtSymbol)), this, SLOT(setSymbol(QwtSymbol)));
}

void PlotSelectionWidget::setStyleColour(const QColor &newColour)
{
    if(newColour.isValid())
    {
        selectedStyleColour = newColour;
        displaySettingsChanged();
    }
}

void PlotSelectionWidget::setStyleColour(const QString& newColourName)
{
    setStyleColour(QColor(newColourName));
}

void PlotSelectionWidget::selectSymbolClicked()
{
    newSymbolDialog->show();
    newSymbolDialog->setFocus();
}

void PlotSelectionWidget::selectStyleColourClicked()
{
    setStyleColour(QColorDialog::getColor(selectedStyleColour,this));
}

void PlotSelectionWidget::setSymbol(QwtSymbol symbol)
{
    selectedSymbol = symbol;
    displaySettingsChanged();
}

void PlotSelectionWidget::curveNamesUpdated(vector<QString> curveNames)
{
    QString cur = curveComboBox->currentText(); //using text handles the potential for a name to be removed
    curveComboBox->clear();

    BOOST_FOREACH(QString& s, curveNames) {
        if(curveComboBox->findText(s) == -1)
            curveComboBox->addItem(s);
    }

    curveComboBox->setCurrentIndex(curveComboBox->findText(cur));
}

void PlotSelectionWidget::setSelectedCurve(QString newCurveName)
{
    curveComboBox->setCurrentIndex(curveComboBox->findText(newCurveName));
}

void PlotSelectionWidget::focusWindowChanged(QMdiSubWindow* focusWindow)
{
    if(focusWindow) {
        static QMdiSubWindow* lastWindow = NULL;
        if(lastWindow != focusWindow) {
            QWidget* widget = focusWindow->widget();
            bool correctWindowType = (typeid(*widget) == typeid(PlotDisplay));

            // Enable if the selected window is the right type, disable otherwise.
            this->setEnabled(correctWindowType);

            currentDisplay = NULL;

            if(correctWindowType)
            {
                // Assign to the current display
                currentDisplay = (PlotDisplay*)widget;

                setSelectedCurve(selectedCurveName);
                // Get the current settings for the selected layers from the new window.
                updateSelectedCurveSettings();
            }
            lastWindow = focusWindow;
        }
    }
}

void PlotSelectionWidget::updateSelectedCurveSettings()
{
    selectedCurveName = curveComboBox->currentText();

    if(currentDisplay) {
        if(currentDisplay->nameExists(selectedCurveName)) {
            curveEnabledCheckBox->setEnabled(true);
            bool enabled = currentDisplay->isCurveEnabled(selectedCurveName);
            // Update controls with stored values.
            curveEnabledCheckBox->setChecked(!enabled);
            curveEnabledCheckBox->toggle();

            if(enabled) {
                //symbol
                selectedSymbol = currentDisplay->getSymbol(selectedCurveName);
                updateButtonSymbol(symbolButton, selectedSymbol);
                //line style
                styleCombo->setCurrentIndex(styleCombo->findText(getStyleName(currentDisplay->getLineStyle(selectedCurveName))));

                selectedStyleColour = currentDisplay->getLineColour(selectedCurveName);
            }
        }
        else {
            curveEnabledCheckBox->setChecked(true);
            curveEnabledCheckBox->toggle();
            curveEnabledCheckBox->setEnabled(false);
        }
    }
}

void PlotSelectionWidget::displaySettingsChanged()
{
    if(currentDisplay) {
        if(currentDisplay->nameExists(selectedCurveName)) {

            updateButtonSymbol(symbolButton, selectedSymbol);
            updateButtonColour(styleColourButton, selectedStyleColour);

            //can add alpha later if required
            //colour.setAlpha(alphaSlider->value());
            currentDisplay->toggleCurve(selectedCurveName, curveEnabledCheckBox->isChecked());

            selectedStyle = getStyleFromInt(styleCombo->currentIndex());

            //assumes curve is already attached to plot (should be)
            currentDisplay->updateCurveProperties(selectedCurveName, selectedSymbol, selectedStyle, selectedStyleColour);
        }
    }
}

void PlotSelectionWidget::updateButtonColour(QToolButton *button, QColor colour)
{
    QPixmap tempPixmap(22,22);
    QPainter painter(&tempPixmap);
    tempPixmap.fill(colour);
    painter.drawRect(0,0,tempPixmap.width(),tempPixmap.height());
    button->setIcon(QIcon(tempPixmap));
    button->setAutoRaise(true);
}

void PlotSelectionWidget::updateButtonSymbol(QToolButton *button, const QwtSymbol& symbol)
{
    if(button != NULL) {
        QPixmap tempPixmap(22,22);
        QPainter painter(&tempPixmap);
        tempPixmap.fill(Qt::lightGray);
        painter.drawRect(0,0,tempPixmap.width(),tempPixmap.height());
        symbol.drawSymbol(&painter, QPointF(tempPixmap.width()*0.5,tempPixmap.height()*0.5));
        QIcon ButtonIcon(tempPixmap);
        button->setIcon(ButtonIcon);
        button->setAutoRaise(true);
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

QString PlotSelectionWidget::getStyleName(QwtPlotCurve::CurveStyle id)
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
