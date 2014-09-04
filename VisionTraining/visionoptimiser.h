#ifndef VISIONOPTIMISER_H
#define VISIONOPTIMISER_H

#include <QMainWindow>
#include "Tools/Optimisation/Optimiser.h"
#include "Vision/VisionWrapper/visioncontrolwrappertraining.h"

//uncomment this for multiple optimisers
#define MULTI_OPT

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
        GOAL_OPT        = 1,
        OBSTACLE_OPT    = 2,
        LINE_OPT        = 3,
        GENERAL_OPT     = 4
    };

    static OPT_TYPE getChoiceFromString(string str);
    static OPT_ID getIDFromInt(int i);
    static std::string getIDName(OPT_ID id);

    explicit VisionOptimiser(QWidget *parent = 0, OPT_TYPE id=PSO);
    ~VisionOptimiser();

    void run(std::string directory, int total_iterations, bool use_ground_errors);
    void gridSearch(std::string directory, int grids_per_side, bool use_ground_errors);
    void errorPandRevaluation(std::string directory, bool use_ground_errors);

private:
    bool trainingStep(int iteration,
                      const std::vector<std::vector<VisionFieldObject *> >& ground_truth,
                      std::ostream& performance_log,
                      const std::string& stream_name,
                      bool use_ground_errors);

    map<OPT_ID, float> evaluateBatch(const std::vector<std::vector<VisionFieldObject *> >& ground_truth,
                                     const std::string& stream_name,
                                     map<VFO_ID, float>& false_pos_costs,
                                     map<VFO_ID, float>& false_neg_costs,
                                     bool use_ground_errors) const;

    map<OPT_ID, pair<double, double> > evaluateBatchPR(const std::vector<std::vector<VisionFieldObject *> >& ground_truth,
                                                       const std::string& stream_name,
                                                       bool ground_errors) const;

    void printResults(int iteration, map<OPT_ID, float> fitnesses, std::ostream& performance_log) const;
    void setupVisionConstants();
    void setupCosts();
    void openLogFiles(std::string directory);
    std::vector<Parameter> getParams(OPT_ID id);

private slots:
    void halt() {m_halted = true;}

private:
    Ui::VisionOptimiser *ui;                //! @var UI pointer

#ifdef MULTI_OPT
    std::vector<Optimiser*> m_optimiser_std::list;        //! @var std::list of optimiser pointers
    map<OPT_ID, Optimiser*> m_optimisers;       //! @var map between object class and optimiser
    map<OPT_ID, float> m_best_fitnesses;             //! @var std::list of best fitnesses seen
    map<OPT_ID, std::vector<Parameter> > m_best_params;   //! @var std::list of best params seen
#else
    Optimiser* m_optimiser;             //! @var optimiser pointer
    float m_best_fitness;               //! @var best fitness seen
    std::vector<Parameter> m_best_params;    //! @var best params seen
#endif

    map<VFO_ID, std::vector<OPT_ID> > m_vfo_optimiser_map;    //! @var map between exact field object type and broad class.
    map<VFO_ID, float> m_false_positive_costs;           //! @var map between field object type and false positive cost.
    map<VFO_ID, float> m_false_negative_costs;           //! @var map between field object type and false negative cost.

    VisionControlWrapper* vision;   //! @var The vision training wrapper.
    std::string m_training_image_name,   //! @var The file name for the training batch.
            m_test_image_name;      //! @var The file name for the test batch.

    std::vector<std::vector<VisionFieldObject *> > m_ground_truth_training;    //! @var labels for the training batch
    std::vector<std::vector<VisionFieldObject *> > m_ground_truth_test;        //! @var labels for the test batch

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

    std::string m_opt_name;                      //! @var optimiser name
};

#endif // VISIONOPTIMISER_H
