#ifndef WALKPARAMETERWIDGET_H
#define WALKPARAMETERWIDGET_H

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

class NUSystem;
class JobList;
class NUIO;
class WalkParameters;


class WalkParameterWidget : public QWidget
{
    Q_OBJECT
public:
    /*!
      @brief Default constructor.
      */
    WalkParameterWidget(QMdiArea* parentMdiWidget, QWidget *parent = 0);
    /*!
      @brief Destructor.
      */
    ~WalkParameterWidget();

public slots:

private slots:
    /*!
      @brief Reads the settings for the currently selected layer and sets
             the current selection to these.
      */
    void walkParameterChanged();
    /*!
      @brief Updates the current layers enabled status.
      */

private:
    QVBoxLayout* overallLayout;                 //!< Overall widget layout.
    
    // Shift Amplitude
    QHBoxLayout* shiftAmplitudeLayout;          //!< Layout for shift amplitude
    QLabel* shiftAmplitudeLabel;                //!< Label for shift amplitude
    QSlider* shiftAmplitudeSlider;              //!< Slider for alpha selection
    QSpinBox* shiftAmplitudeSpinBox;            //!< SpinBox for alpha selection
    
    // Shift Frequency
    QHBoxLayout* shiftFrequencyLayout;          //!< Layout for shift frequency
    QLabel* shiftFrequencyLabel;                //!< Label for shift frequency
    QSlider* shiftFrequencySlider;              //!< Slider for alpha selection
    QSpinBox* shiftFrequencySpinBox;            //!< SpinBox for alpha selection
    
    // Phase offset
    QHBoxLayout* phaseOffsetLayout;             //!< Layout for phase offset
    QLabel* phaseOffsetLabel;                   //!< Label for phase offset
    QSlider* phaseOffsetSlider;                 //!< Slider for phase offset
    QSpinBox* phaseOffsetSpinBox;               //!< SpinBox for phase offset
    
    // Phase Reset Offset
    QHBoxLayout* phaseResetLayout;             //!< Layout for phase offset
    QLabel* phaseResetLabel;                   //!< Label for phase offset
    QSlider* phaseResetSlider;                 //!< Slider for phase offset
    QSpinBox* phaseResetSpinBox;               //!< SpinBox for phase offset
    
    // X Speed
    QHBoxLayout* xSpeedLayout;                 //!< Layout for step size
    QLabel* xSpeedLabel;                       //!< Label for step size
    QSlider* xSpeedSlider;                     //!< Slider for alpha selection
    QSpinBox* xSpeedSpinBox;                   //!< SpinBox for alpha selection
    
    // Y Speed
    QHBoxLayout* ySpeedLayout;                 //!< Layout for step size
    QLabel* ySpeedLabel;                       //!< Label for step size
    QSlider* ySpeedSlider;                     //!< Slider for alpha selection
    QSpinBox* ySpeedSpinBox;                   //!< SpinBox for alpha selection
    
    // Yaw Speed
    QHBoxLayout* yawSpeedLayout;               //!< Layout for step size
    QLabel* yawSpeedLabel;                     //!< Label for step size
    QSlider* yawSpeedSlider;                   //!< Slider for alpha selection
    QSpinBox* yawSpeedSpinBox;                 //!< SpinBox for alpha selection
    
    void createWidgets();       //!< Create all of the child widgets.
    void createLayout();        //!< Layout all of the child widgets.
    void createConnections();   //!< Connect all of the child widgets.
    bool disableWriting;        //!< Flag used to disable the writing of settings back to the layers when updating the displays.
    
    NUSystem* m_nusystem;
    NUIO* m_io;
    JobList* m_job_list; 
    WalkParameters* m_walk_parameters;

    public:



};

#endif // LAYERSELECTIONWIDGET_H
