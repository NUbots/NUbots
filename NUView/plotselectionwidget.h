#ifndef PLOTSELECTIONWIDGET_H
#define PLOTSELECTIONWIDGET_H

#include <QWidget>
#include <QStringList>
#include <QVector>
#include <QPair>
#include <QList>
#include <qwt/qwt_symbol.h>

#include "plotdisplay.h"

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
      @brief Get the currently selected colour.
      @return The current colour of the selected layer.
      */
    QColor getSelectedSymbolColour();
    QColor getSelectedStyleColour();

    /*!
      @brief Get the currently selected layer.
      @return The id of the selected layer
      */
    QString getSelectedCurve();

    /*!
      @brief Get a displays history list index.
      @param display The display whose history is required.
      @return The list index of the displays history. -1 if it does not have any.
      */
    int getDisplayHistoryIndex(PlotDisplay *display);

public slots:
    /*!
      @brief Set the currently selected colour.
      @param newColour The new colour.
      */
    void setSymbolColour(const QColor& newColour);
    void setStyleColour(const QColor& newColour);
    /*!
      @brief Set the currently selected colour.
      @param newColourName The name of the new colour.
      */
    void setSymbolColour(const QString& newColourName);
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
    void symbolSettingsChanged();
    void styleSettingsChanged();
    /*!
      @brief Updates the current layers enabled status.
      */
    void enabledSettingChanged(bool enabled);
    /*!
      @brief Updates the current layers primary status.
      */
    void primarySettingChanged(bool primary);
    /*!
      @brief Opens a dialog to allow selection of a custom colour.
      If a valid colour is selected this is updated as the current selection.
      */
    void selectSymbolColourClicked();
    void selectStyleColourClicked();

private:
    static QwtSymbol::Style getSymbolFromInt(int i);
    static QwtPlotCurve::CurveStyle getStyleFromInt(int i);
    static QString getSymbolName(QwtSymbol::Style id);
    static QString getStyleName(QwtPlotCurve::CurveStyle id);

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
    QPushButton* symbolColourButton;            //!< Label for colour selection.
    QLabel* styleLabel;            //!< Label for line style selection.
    QPushButton* styleColourButton;            //!< Label for colour selection.

    // Combos
    QComboBox* symbolCombo;
    QComboBox* styleCombo;

    PlotDisplay* currentDisplay;      //!< Pointer to the current display window.

    QColor selectedSymbolColour;      //!< Storage of the currently selected colour.
    QColor selectedStyleColour;      //!< Storage of the currently selected colour.


    void createWidgets();       //!< Create all of the child widgets.
    void createLayout();        //!< Layout all of the child widgets.
    void createConnections();   //!< Connect all of the child widgets.
    bool disableWriting;        //!< Flag used to disable the writing of settings back to the layers when updating the displays.
    QList< QPair<PlotDisplay*,QString> > selectedCurveHistory;
};

#endif // PLOTSELECTIONWIDGET_H
