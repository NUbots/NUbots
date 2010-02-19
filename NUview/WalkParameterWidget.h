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
    
    // Step Size
    QHBoxLayout* stepSizeLayout;                //!< Layout for step size
    QLabel* stepSizeLabel;                      //!< Label for step size
    QSlider* stepSizeSlider;                    //!< Slider for alpha selection
    QSpinBox* stepSizeSpinBox;                  //!< SpinBox for alpha selection
    

    void createWidgets();       //!< Create all of the child widgets.
    void createLayout();        //!< Layout all of the child widgets.
    void createConnections();   //!< Connect all of the child widgets.
    bool disableWriting;        //!< Flag used to disable the writing of settings back to the layers when updating the displays.
    
    NUSystem* m_nusystem;
    NUIO* m_io;
    JobList* m_job_list; 

    public:



};

#endif // LAYERSELECTIONWIDGET_H
