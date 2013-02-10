#ifndef LAYERSELECTIONWIDGET_H
#define LAYERSELECTIONWIDGET_H

#include <QWidget>
#include <QStringList>
#include <QVector>
#include <QPair>
#include <QList>


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
class GLDisplay;


class LayerSelectionWidget : public QWidget
{
    Q_OBJECT
public:
    /*!
      @brief Default constructor.
      */
    LayerSelectionWidget(QMdiArea* parentMdiWidget, QWidget *parent = 0);
    /*!
      @brief Destructor.
      */
    ~LayerSelectionWidget();
    /*!
      @brief Get the currently selected colour.
      @return The current colour of the selected layer.
      */
    QColor getSelectedColour();

    /*!
      @brief Get the currently selected layer.
      @return The id of the selected layer
      */
    int getSelectedLayer();

    /*!
      @brief Get a displays history list index.
      @param display The display whose history is required.
      @return The list index of the displays history. -1 if it does not have any.
      */
    int getDisplayHistoryIndex(GLDisplay* display);

public slots:
    /*!
      @brief Set the currently selected colour.
      @param newColour The new colour.
      */
    void setColour(const QColor& newColour);
    /*!
      @brief Set the currently selected colour.
      @param newColourName The name of the new colour.
      */
    void setColour(const QString& newColourName);
    /*!
      @brief Set the currently selected layer.
      @param The id of the layer to select.
      */
    void setSelectedLayer(int newLayerId);

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
    void updateSelectedLayerSettings();
    /*!
      @brief Refreshes the display of the current colour selection and if
      required writes it to the current layer settings.
      */
    void colourSettingsChanged();
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
    void selectCustomColourClicked();

private:
    QStringList quickSelectColoursList; //!< List to store the names of the quick select colours.
    QMdiArea* mdiWidget; //!< Pointer to keep track of the mdi parent area.

    // Layouts
    QHBoxLayout* layerSelectionlayout;          //!< Layout for layer selection.
    QGridLayout* quickColourSelectionLayout;    //!< Layout for quick colour selection.
    QHBoxLayout* colourSelectionLayout;         //!< Layout for colour selection.
    QHBoxLayout* alphaSelectionlayout;          //!< Layout for alpha selection.
    QVBoxLayout* overallLayout;                 //!< Overall widget layout.

    // Layer selection combo box and checkboxes
    QComboBox* layerComboBox;           //!< Combobox for layer names.
    QCheckBox* layerEnabledCheckBox;    //!< Checkbox to toggle layer enabled.
    QCheckBox* layerPrimaryCheckBox;    //!< Checkbox to set layer to primary.

    // Labels
    QLabel* colourLabel;            //!< Label for colour selection.
    QLabel* drawingColourLabel;     //!< Label for current selected colour display.
    QLabel* alphaLabel;             //!< Label for alpha selection

    // Sliders
    QSlider* alphaSlider;       //!< Slider for alpha selection.
    QSpinBox* alphaSpinBox;     //!< SpinBox for alpha selection.

    // Push Buttons
    QVector<QToolButton*> quickSelectPushButtons;   //!< Quick select colour buttons.
    QPushButton* selectCustomColourPushButton;      //!< Custom colour selection button.

    // Signal mapper
    QSignalMapper *quickSelectSignalMapping;    //!< Signal mapper to combine quick select colour buttons.

    GLDisplay* currentDisplay;      //!< Pointer to the current display window.

    void createWidgets();       //!< Create all of the child widgets.
    void createLayout();        //!< Layout all of the child widgets.
    void createConnections();   //!< Connect all of the child widgets.
    bool disableWriting;        //!< Flag used to disable the writing of settings back to the layers when updating the displays.
    QColor selectedColour;      //!< Storage of the currently selected colour.
    QList< QPair<GLDisplay*,int> > selectedLayerHistory;

public:
    const int quickSelectColoursPerLine;    //!< The maximum number of quick select buttons per row.



};

#endif // LAYERSELECTIONWIDGET_H
