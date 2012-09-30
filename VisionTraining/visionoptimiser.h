#ifndef VISIONOPTIMISER_H
#define VISIONOPTIMISER_H

#include <QMainWindow>
#include "Tools/Optimisation/Optimiser.h"
#include "Vision/VisionWrapper/visioncontrolwrappertraining.h"

namespace Ui {
class VisionOptimiser;
}

class VisionOptimiser : public QMainWindow
{
    Q_OBJECT
    
public:
    enum OPT_ID {
        EHCLS,
        PGRL,
        PSO
    };

    static OPT_ID getChoiceFromQString(QString str);

    explicit VisionOptimiser(QWidget *parent = 0, OPT_ID id = PSO);
    ~VisionOptimiser();

    void run(string directory, int total_iterations);
private:
    int runTrainingStep(int iteration, bool individual = false);
    int runEvaluationStep(int iteration, bool individual = false);
    void setupVisionConstants();

private slots:
    void halt() {m_halted = true;}

private:
    Ui::VisionOptimiser *ui;
    Optimiser* m_optimiser;
    VisionControlWrapper* vision;
    string m_train_image_name,
            m_test_image_name;
    vector<vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > > > m_ground_truth_training;
    vector<vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > > > m_ground_truth_test;
    bool m_halted;
    //! LOGS
    ofstream m_progress_log,
             m_optimiser_log,
             m_training_performance_log,
             m_test_performance_log;
};

#endif // VISIONOPTIMISER_H
