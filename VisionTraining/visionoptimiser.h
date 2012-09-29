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

    explicit VisionOptimiser(OPT_ID id = PSO, QWidget *parent = 0);
    ~VisionOptimiser();

    void run(string directory, int total_iterations);
private:
    int runOptimisationStep(int iteration, bool individual = false);

private slots:
    void halt() {m_halted = true;}

private:
    Ui::VisionOptimiser *ui;
    Optimiser* m_optimiser;
    VisionControlWrapper* vision;
    vector<vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > > > m_ground_truth_full;
    bool m_halted;
    //! LOGS
    ofstream m_progress_log,
             m_performance_log,
             m_optimiser_log;
};

#endif // VISIONOPTIMISER_H
