#ifndef VISIONOPTIMISER_H
#define VISIONOPTIMISER_H

#include <QMainWindow>
#include "Tools/Optimisation/Optimiser.h"
#include "Vision/VisionWrapper/visioncontrolwrappertraining.h"

//#define MULTI_OPT

namespace Ui {
class VisionOptimiser;
}

class VisionOptimiser : public QMainWindow
{
    Q_OBJECT
    
public:
    enum OPT_TYPE {
        EHCLS,
        PGRL,
        PSO,
        PGA
    };

    enum OPT_ID {
        BALL_OPT        = 0,
        GOAL_BEACON_OPT = 1,
        OBSTACLE_OPT    = 2,
        LINE_OPT        = 3,
        GENERAL_OPT     = 4
    };

    static OPT_TYPE getChoiceFromString(string str);
    static OPT_ID getIDFromInt(int i);
    static string getIDName(OPT_ID id);

    explicit VisionOptimiser(QWidget *parent = 0, OPT_TYPE id=PSO);
    ~VisionOptimiser();

    void run(string directory, int total_iterations);
    void gridSearch(string directory, int grids_per_side);
    void errorPandRevaluation(string directory);

private:
    bool trainingStep(int iteration,
                      const vector<vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > > >& ground_truth,
                      ostream &performance_log, const string &stream_name);

    map<OPT_ID, float> evaluateBatch(const vector<vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > > >& ground_truth,
                                     const string& stream_name,
                                     map<VisionFieldObject::VFO_ID, float>& false_pos_costs,
                                     map<VisionFieldObject::VFO_ID, float>& false_neg_costs) const;

    map<OPT_ID, pair<double, double> > evaluateBatchPR(const vector<vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > > >& ground_truth,
                                                       const string& stream_name) const;

    void printResults(int iteration, map<OPT_ID, float> fitnesses, ostream& performance_log) const;
    void setupVisionConstants();
    void setupCosts();

private slots:
    void halt() {m_halted = true;}

private:
    Ui::VisionOptimiser *ui;

#ifdef MULTI_OPT
    vector<Optimiser*> m_optimiser_list;
    map<OPT_ID, Optimiser*> m_optimisers;
#else
    Optimiser* m_optimiser;
#endif

    map<VisionFieldObject::VFO_ID, vector<OPT_ID> > m_vfo_optimiser_map;
    map<VisionFieldObject::VFO_ID, float> m_false_positive_costs;
    map<VisionFieldObject::VFO_ID, float> m_false_negative_costs;
    VisionControlWrapper* vision;
    string m_training_image_name,
            m_test_image_name;
    vector<vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > > > m_ground_truth_training;
    vector<vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > > > m_ground_truth_test;
    bool m_halted;
    //! LOGS
#ifdef MULTI_OPT
    map<OPT_ID, ofstream*> m_optimiser_logs;
    map<OPT_ID, ofstream*> m_individual_progress_logs;
#else
    ofstream m_progress_log,
             m_optimiser_log;
#endif
    ofstream m_training_performance_log,
             m_test_performance_log;

    string m_opt_name;
};

#endif // VISIONOPTIMISER_H
