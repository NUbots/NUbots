#ifndef LAYERSELECTIONWIDGET_H
#define LAYERSELECTIONWIDGET_H

#include <QWidget>
#include <QStringList>

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
class QPicture;
class GLDisplay;

class LayerSelectionWidget : public QWidget
{
    Q_OBJECT
public:
    LayerSelectionWidget(QMdiArea* parentMdiWidget, QWidget *parent = 0);
    ~LayerSelectionWidget();
    
    QStringList coloursList;

    QMdiArea* mdiWidget;
    QHBoxLayout* layerSelectionlayout;
    QHBoxLayout* alphaSelectionlayout;
    QGridLayout* colourSelectionLayout;
    QVBoxLayout* overallLayout;

    QComboBox* layerComboBox;
    QCheckBox* layerEnabledCheckBox;
    QCheckBox* layerPrimaryCheckBox;

    // Labels
    QLabel* colourLabel;
    QLabel* drawingColourLabel;
    QLabel* alphaLabel;

    // Sliders
    QSlider* colourSlider;
    QSlider* alphaSlider;

    QSpinBox* colourSpinBox;
    QSpinBox* alphaSpinBox;

    QColor getSelectedColour();

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
    QString getClosestColourName(QColor colour);

};

#endif // LAYERSELECTIONWIDGET_H
