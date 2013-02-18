#ifndef PLOTSELECTIONWIDGET_H
#define PLOTSELECTIONWIDGET_H

#include <QWidget>
#include <QStringList>
#include <QVector>
#include <QPair>
#include <QList>
#include <qwt/qwt_symbol.h>

#include "plotdisplay.h"
#include "createqwtsymboldialog.h"

class QMdiArea;
class QComboBox;
class QCheckBox;
class QLabel;
class QSlider;
class QSpinBox;
class QVBoxLayout;
class QHBoxLayout;
class QGridLayout;
class QMdiSubWindow;
class QPushButton;
class QSignalMapper;
class QToolButton;


class PlotSelectionWidget : public QWidget
{
    Q_OBJECT
public:
    /*!
      @brief Default constructor.
      */
    PlotSelectionWidget(QMdiArea* parentMdiWidget, QWidget *parent = 0);
    /*!
      @brief Destructor.
      */
    ~PlotSelectionWidget();

    /*!
      @brief Get a displays history list index.
      @param display The display whose history is required.
      @return The list index of the displays history. -1 if it does not have any.
      */
    int getDisplayHistoryIndex(PlotDisplay *display);

public slots:
    void curveNamesUpdated(vector<QString> curveNames);
    /*!
      @brief Set the currently selected colour.
      @param newColour The new colour.
      */
    void setStyleColour(const QColor& newColour);
    /*!
      @brief Set the currently selected colour.
      @param newColourName The name of the new colour.
      */
    void setStyleColour(const QString& newColourName);
    /*!
      @brief Set the currently selected layer.
      @param The id of the layer to select.
      */
    void setSelectedCurve(QString newCurveName);

private slots:
    /*!
      @brief Selects a new focus window.
      @param focusWindow The new window of focus.
      */
    void focusWindowChanged(QMdiSubWindow* focusWindow);
    /*!
      @brief Reads the settings for the currently selected layer and sets
             the current selection to these.
      */
    void updateSelectedCurveSettings();
    /*!
      @brief Refreshes the display of the current colour selection and if
      required writes it to the current layer settings.
      */
    void displaySettingsChanged();
    /*!
      @brief Opens a dialog to allow selection of a custom colour.
      If a valid colour is selected this is updated as the current selection.
      */
    void selectSymbolClicked();
    void selectStyleColourClicked();

    void setSymbol(QwtSymbol symbol);

private:
    static void updateButtonColour(QToolButton *button, QColor colour);
    static void updateButtonSymbol(QToolButton* button, const QwtSymbol& symbol);
    static QwtPlotCurve::CurveStyle getStyleFromInt(int i);
    static QString getStyleName(QwtPlotCurve::CurveStyle id);
    void createWidgets();       //!< Create all of the child widgets.
    void createLayout();        //!< Layout all of the child widgets.
    void createConnections();   //!< Connect all of the child widgets.

    QMdiArea* mdiWidget; //!< Pointer to keep track of the mdi parent area.

    // Layouts
    QHBoxLayout* curveSelectionlayout;          //!< Layout for layer selection.
    QHBoxLayout* symbolSelectionLayout;         //!< Layout for symbol selection.
    QHBoxLayout* styleSelectionLayout;         //!< Layout for style selection.
    QVBoxLayout* overallLayout;                 //!< Overall widget layout.

    // Curve Options
    QComboBox* curveComboBox;           //!< Combobox for layer names.
    QCheckBox* curveEnabledCheckBox;    //!< Checkbox to toggle layer enabled.

    // Labels
    QLabel* symbolLabel;            //!< Label for colour selection.
    QLabel* styleLabel;            //!< Label for line style selection.

    // Buttons
    QToolButton* symbolButton;            //!< Label for colour selection.
    QToolButton* styleColourButton;            //!< Label for colour selection.

    // Combos
    QComboBox* styleCombo;

    QString selectedCurveName;

    PlotDisplay* currentDisplay;      //!< Pointer to the current display window.

    QwtSymbol selectedSymbol;
    CreateQwtSymbolDialog* newSymbolDialog;
    QwtPlotCurve::CurveStyle selectedStyle;
    QColor selectedStyleColour;      //!< Storage of the currently selected colour.
};

#endif // PLOTSELECTIONWIDGET_H
