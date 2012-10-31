#ifndef VISIONOPTIMISER_H
#define VISIONOPTIMISER_H

#include <QMainWindow>
#include "Tools/Optimisation/Optimiser.h"
#include "Vision/VisionWrapper/visioncontrolwrappertraining.h"

//uncomment this for multiple optimisers
//#define MULTI_OPT

namespace Ui {
class VisionOptimiser;
}

class VisionOptimiser : public QMainWindow
{
    Q_OBJECT
    
public:
    //! Represents the type of optimiser chosen
    enum OPT_TYPE {
        EHCLS,
        PGRL,
        PSO,
        PGA
    };

    //! Represents broad classes of field object
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
    void openLogFiles(string directory);

private slots:
    void halt() {m_halted = true;}

private:
    Ui::VisionOptimiser *ui;                //! @var UI pointer

#ifdef MULTI_OPT
    vector<Optimiser*> m_optimiser_list;    //! @var list of optimiser pointers
    map<OPT_ID, Optimiser*> m_optimisers;   //! @var map between object class and optimiser
#else
    Optimiser* m_optimiser;                 //! @var optimiser pointer
#endif

    map<VisionFieldObject::VFO_ID, vector<OPT_ID> > m_vfo_optimiser_map;    //! @var map between exact field object type and broad class.
    map<VisionFieldObject::VFO_ID, float> m_false_positive_costs;           //! @var map between field object type and false positive cost.
    map<VisionFieldObject::VFO_ID, float> m_false_negative_costs;           //! @var map between field object type and false negative cost.

    VisionControlWrapper* vision;   //! @var The vision training wrapper.
    string m_training_image_name,   //! @var The file name for the training batch.
            m_test_image_name;      //! @var The file name for the test batch.

    vector<vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > > > m_ground_truth_training;    //! @var labels for the training batch
    vector<vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > > > m_ground_truth_test;        //! @var labels for the test batch

    bool m_halted;                  //! @var A flag for the user opting to halt.
    //! LOGS
#ifdef MULTI_OPT
    map<OPT_ID, ofstream*> m_optimiser_logs;            //! @var map between object class and optimiser state log
    map<OPT_ID, ofstream*> m_individual_progress_logs;  //! @var map between object class and parameter log
#else
    ofstream m_optimiser_log,               //! @var optimiser state log
             m_progress_log;                //! @var parameter log

#endif
    ofstream m_training_performance_log,    //! @var training fitness log
             m_test_performance_log;        //! @var test fitness log

    string m_opt_name;                      //! @var optimiser name
};

#endif // VISIONOPTIMISER_H
