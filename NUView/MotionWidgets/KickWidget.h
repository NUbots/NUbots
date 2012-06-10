#ifndef KICKWIDGET_H
#define KICKWIDGET_H

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

class JobList;


class KickWidget : public QWidget
{
    Q_OBJECT
public:
    /*!
      @brief Default constructor.
      */
    KickWidget(QWidget *parent = 0);
    /*!
      @brief Destructor.
      */
    ~KickWidget();

public slots:

private slots:
    /*!
      @brief Called every time the kick button is pressed
      */
    void kickPressed();

private:
    QVBoxLayout* overallLayout;                 //!< Overall widget layout.
    
    // Kick Position
    QHBoxLayout* positionLayout;                //!< Layout for kick position
    QLabel* positionLabel;                      //!< Label for kick position
    QSpinBox* positionXSpinBox;                 //!< SpinBox for kick position X
    QSpinBox* positionYSpinBox;                 //!< SpinBox for kick position Y
    
    // Kick Target
    QHBoxLayout* targetLayout;                  //!< Layout for target position
    QLabel* targetLabel;                        //!< Label for target position
    QSpinBox* targetXSpinBox;                   //!< SpinBox for target position X
    QSpinBox* targetYSpinBox;                   //!< SpinBox for target position Y
    
    // Kick Button
    QHBoxLayout* kickButtonLayout;              //!< Laout fo the kick button
    QPushButton* kickButton;                    //!< Button to trigger sending of kick job to robot
    
    

    void createWidgets();       //!< Create all of the child widgets.
    void createLayout();        //!< Layout all of the child widgets.
    void createConnections();   //!< Connect all of the child widgets.
    bool disableWriting;        //!< Flag used to disable the writing of settings back to the layers when updating the displays.
    
    JobList* m_job_list; 

    public:



};

#endif // KICKWIDGET_H
