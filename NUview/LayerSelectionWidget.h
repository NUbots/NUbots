#ifndef LAYERSELECTIONWIDGET_H
#define LAYERSELECTIONWIDGET_H

#include <QWidget>

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
class GLDisplay;

class LayerSelectionWidget : public QWidget
{
    Q_OBJECT
public:
    LayerSelectionWidget(QMdiArea* parentMdiWidget, QWidget *parent = 0);
    ~LayerSelectionWidget();

    QMdiArea* mdiWidget;
    QHBoxLayout* layerSelectionlayout;
    QGridLayout* colourSelectionLayout;
    QVBoxLayout* overallLayout;

    QComboBox* layerComboBox;
    QCheckBox* layerEnabledCheckBox;
    QCheckBox* layerPrimaryCheckBox;

    // Labels
    QLabel* redLabel;
    QLabel* greenLabel;
    QLabel* blueLabel;
    QLabel* alphaLabel;

    // Sliders
    QSlider* redSlider;
    QSlider* greenSlider;
    QSlider* blueSlider;
    QSlider* alphaSlider;

    QSpinBox* redSpinBox;
    QSpinBox* greenSpinBox;
    QSpinBox* blueSpinBox;
    QSpinBox* alphaSpinBox;

private slots:
    void isPrimaryChanged(bool isPrimary);
    void focusWindowChanged(QMdiSubWindow* focusWindow);
    void updateSelectedLayerSettings();
    void colourSettingsChanged();
    void enabledSettingChanged(bool enabled);
    void primarySettingChanged(bool primary);

private:
    GLDisplay* currentDisplay;
    void createWidgets();
    void createLayout();
    void createConnections();
    bool disableWriting;

};

#endif // LAYERSELECTIONWIDGET_H
