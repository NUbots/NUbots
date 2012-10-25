#include "visionoptimiser.h"
#include "ui_visionoptimiser.h"

#include "Tools/Optimisation/EHCLSOptimiser.h"
#include "Tools/Optimisation/PGRLOptimiser.h"
#include "Tools/Optimisation/PSOOptimiser.h"
#include "Tools/Optimisation/PGAOptimiser.h"
#include "Vision/visionconstants.h"
#include "optimiserselectwindow.h"
#include <QMessageBox>

VisionOptimiser::OPT_TYPE VisionOptimiser::getChoiceFromString(string str)
{
    if(str.compare("PGRL") == 0) {
        return PGRL;
    }
    else if(str.compare("EHCLS") == 0) {
        return EHCLS;
    }
    else if(str.compare("PGA") == 0) {
        return PGA;
    }
    else {
        return PSO;
    }
}

VisionOptimiser::OPT_ID VisionOptimiser::getIDFromInt(int i)
{
    switch(i) {
    case 0: return BALL_OPT;
    case 1: return GOAL_BEACON_OPT;
    case 2: return OBSTACLE_OPT;
    case 3: return LINE_OPT;
    case 4: return GENERAL_OPT;
    }
}

VisionOptimiser::VisionOptimiser(QWidget* parent, OPT_TYPE id) :
    QMainWindow(parent),
    ui(new Ui::VisionOptimiser)
{
    ui->setupUi(this);
    QObject::connect(ui->haltPB, SIGNAL(clicked()), this, SLOT(halt()));

    //put dialog here for selecting multi/single and which single opts
    setupCosts();

    switch(id) {
    case EHCLS:
        m_opt_name = "VisionEHCLS";
#ifdef MULTI_OPT
        m_optimisers[BALL_OPT] = new EHCLSOptimiser("EHCLSBall", VisionConstants::getBallParams());
        m_optimisers[GOAL_BEACON_OPT] = new EHCLSOptimiser("EHCLSGoalBeacon", VisionConstants::getGoalBeaconParams());
        m_optimisers[OBSTACLE_OPT] = new EHCLSOptimiser("EHCLSObstacle", VisionConstants::getObstacleParams());
        m_optimisers[LINE_OPT] = new EHCLSOptimiser("EHCLSLine", VisionConstants::getLineParams());
        m_optimisers[GENERAL_OPT] = new EHCLSOptimiser("EHCLSGeneral", VisionConstants::getGeneralParams());
#else
        m_optimiser = new EHCLSOptimiser("VisionEHCLS", VisionConstants::getAllOptimisable());
#endif
        break;
    case PGRL:
        m_opt_name = "VisionPGRL";
#ifdef MULTI_OPT
        m_optimisers[BALL_OPT] = new PGRLOptimiser("PGRLBall", VisionConstants::getBallParams());
        m_optimisers[GOAL_BEACON_OPT] = new PGRLOptimiser("PGRLGoalBeacon", VisionConstants::getGoalBeaconParams());
        m_optimisers[OBSTACLE_OPT] = new PGRLOptimiser("PGRLObstacle", VisionConstants::getObstacleParams());
        m_optimisers[LINE_OPT] = new PGRLOptimiser("PGRLLine", VisionConstants::getLineParams());
        m_optimisers[GENERAL_OPT] = new PGRLOptimiser("PGRLGeneral", VisionConstants::getGeneralParams());
#else
        m_optimiser = new PGRLOptimiser("VisionPGRL", VisionConstants::getAllOptimisable());
#endif
        break;
    case PSO:
        m_opt_name = "VisionPSO";
#ifdef MULTI_OPT
        m_optimisers[BALL_OPT] = new PSOOptimiser("PSOBall", VisionConstants::getBallParams());
        m_optimisers[GOAL_BEACON_OPT] = new PSOOptimiser("PSOGoalBeacon", VisionConstants::getGoalBeaconParams());
        m_optimisers[OBSTACLE_OPT] = new PSOOptimiser("PSOObstacle", VisionConstants::getObstacleParams());
        m_optimisers[LINE_OPT] = new PSOOptimiser("PSOLine", VisionConstants::getLineParams());
        m_optimisers[GENERAL_OPT] = new PSOOptimiser("PSOGeneral", VisionConstants::getGeneralParams());
#else
        m_optimiser = new PSOOptimiser("VisionPSO", VisionConstants::getAllOptimisable());
#endif
        break;
    case PGA:
        m_opt_name = "VisionPGA";
#ifdef MULTI_OPT
        m_optimisers[BALL_OPT] = new PGAOptimiser("PGABall", VisionConstants::getBallParams());
        m_optimisers[GOAL_BEACON_OPT] = new PGAOptimiser("PGAGoalBeacon", VisionConstants::getGoalBeaconParams());
        m_optimisers[OBSTACLE_OPT] = new PGAOptimiser("PGAObstacle", VisionConstants::getObstacleParams());
        m_optimisers[LINE_OPT] = new PGAOptimiser("PGALine", VisionConstants::getLineParams());
        m_optimisers[GENERAL_OPT] = new PGAOptimiser("PGAGeneral", VisionConstants::getGeneralParams());
#else
        m_optimiser = new PGAOptimiser("VisionPGA", VisionConstants::getAllOptimisable());
#endif
        break;
    }


#ifdef MULTI_OPT
    m_vfo_optimiser_map[VisionFieldObject::BALL].push_back(BALL_OPT); m_vfo_optimiser_map[VisionFieldObject::BALL].push_back(GENERAL_OPT);
    m_vfo_optimiser_map[VisionFieldObject::GOAL_Y_L].push_back(GOAL_BEACON_OPT); m_vfo_optimiser_map[VisionFieldObject::GOAL_Y_L].push_back(GENERAL_OPT);
    m_vfo_optimiser_map[VisionFieldObject::GOAL_Y_R].push_back(GOAL_BEACON_OPT); m_vfo_optimiser_map[VisionFieldObject::GOAL_Y_R].push_back(GENERAL_OPT);
    m_vfo_optimiser_map[VisionFieldObject::GOAL_Y_U].push_back(GOAL_BEACON_OPT); m_vfo_optimiser_map[VisionFieldObject::GOAL_Y_U].push_back(GENERAL_OPT);
    m_vfo_optimiser_map[VisionFieldObject::GOAL_B_L].push_back(GOAL_BEACON_OPT); m_vfo_optimiser_map[VisionFieldObject::GOAL_B_L].push_back(GENERAL_OPT);
    m_vfo_optimiser_map[VisionFieldObject::GOAL_B_R].push_back(GOAL_BEACON_OPT); m_vfo_optimiser_map[VisionFieldObject::GOAL_B_R].push_back(GENERAL_OPT);
    m_vfo_optimiser_map[VisionFieldObject::GOAL_B_U].push_back(GOAL_BEACON_OPT); m_vfo_optimiser_map[VisionFieldObject::GOAL_B_U].push_back(GENERAL_OPT);
    m_vfo_optimiser_map[VisionFieldObject::BEACON_Y].push_back(GOAL_BEACON_OPT); m_vfo_optimiser_map[VisionFieldObject::BEACON_Y].push_back(GENERAL_OPT);
    m_vfo_optimiser_map[VisionFieldObject::BEACON_B].push_back(GOAL_BEACON_OPT); m_vfo_optimiser_map[VisionFieldObject::BEACON_B].push_back(GENERAL_OPT);
    m_vfo_optimiser_map[VisionFieldObject::BEACON_U].push_back(GOAL_BEACON_OPT); m_vfo_optimiser_map[VisionFieldObject::BEACON_U].push_back(GENERAL_OPT);
    m_vfo_optimiser_map[VisionFieldObject::OBSTACLE].push_back(OBSTACLE_OPT); m_vfo_optimiser_map[VisionFieldObject::OBSTACLE].push_back(GENERAL_OPT);
    m_vfo_optimiser_map[VisionFieldObject::FIELDLINE].push_back(LINE_OPT); m_vfo_optimiser_map[VisionFieldObject::FIELDLINE].push_back(GENERAL_OPT);
#endif

    vision = VisionControlWrapper::getInstance();
}

VisionOptimiser::~VisionOptimiser()
{
    delete ui;
#ifdef MULTI_OPT
    delete m_optimisers[OBSTACLE_OPT];
    delete m_optimisers[BALL_OPT];
    delete m_optimisers[GOAL_BEACON_OPT];
    delete m_optimisers[LINE_OPT];
    delete m_optimisers[GENERAL_OPT];
#else
    delete m_optimiser;
#endif
}

void VisionOptimiser::run(string directory, int total_iterations)
{
    int iteration = 0;
    int err_code = 0;
    m_halted = false;
    //image streams
    m_train_image_name = directory + string("train_image.strm");
    m_test_image_name = directory + string("test_image.strm");

    //read labels
    ifstream train_label_file((directory + string("train_labels.strm")).c_str());
    ifstream test_label_file((directory + string("test_labels.strm")).c_str());

    total_iterations*=2; //double since we are running once each for training and testing

    //set up logs
#ifdef MULTI_OPT
    m_optimiser_logs[BALL_OPT] = new ofstream((directory + m_opt_name + string("_ball.log")).c_str());
    m_optimiser_logs[GOAL_BEACON_OPT] = new ofstream((directory + m_opt_name + string("_goalbeacon.log")).c_str());
    m_optimiser_logs[OBSTACLE_OPT] = new ofstream((directory + m_opt_name + string("_obstacle.log")).c_str());
    m_optimiser_logs[LINE_OPT] = new ofstream((directory + m_opt_name + string("_line.log")).c_str());
    m_optimiser_logs[GENERAL_OPT] = new ofstream((directory + m_opt_name + string("_general.log")).c_str());

    m_individual_progress_logs[BALL_OPT] = new ofstream((directory + m_opt_name + string("_ball_progress.log")).c_str());
    m_individual_progress_logs[GOAL_BEACON_OPT] = new ofstream((directory + m_opt_name + string("_goalbeacon_progress.log")).c_str());
    m_individual_progress_logs[OBSTACLE_OPT] = new ofstream((directory + m_opt_name + string("_obstacle_progress.log")).c_str());
    m_individual_progress_logs[LINE_OPT] = new ofstream((directory + m_opt_name + string("_line_progress.log")).c_str());
    m_individual_progress_logs[GENERAL_OPT] = new ofstream((directory + m_opt_name + string("_general_progress.log")).c_str());
#else
    m_progress_log.open((directory + m_opt_name + string("_progress.log")).c_str());
    m_optimiser_log.open((directory + m_opt_name + string(".log")).c_str());
#endif

    m_training_performance_log.open((directory + m_opt_name + string("_training_performance.log")).c_str());
    m_training_performance_log.setf(ios_base::fixed);
    m_test_performance_log.open((directory + m_opt_name + string("_test_performance.log")).c_str());
    m_test_performance_log.setf(ios_base::fixed);

    m_training_performance_log << "Iteration Ball GoalBeacon Obstacle Line General" << endl;
    m_test_performance_log << "Iteration Ball GoalBeacon Obstacle Line General" << endl;

    //read in the training and testing labels
    if(!vision->readLabels(train_label_file, m_ground_truth_training)) {
        QMessageBox::warning(this, "Failure", QString("Failed to read label stream: ") + QString((directory + string("train_label.strm")).c_str()));
        return;
    }
    if(!vision->readLabels(test_label_file, m_ground_truth_test)) {
        QMessageBox::warning(this, "Failure", QString("Failed to read label stream: ") + QString((directory + string("test_label.strm")).c_str()));
        return;
    }

    //init gui
    ui->progressBar_opt->setMaximum(total_iterations*2);
    QApplication::processEvents();

    //initialise vision system
    vision->setLUT(directory+string("default.lut"));

    //set the options we need
    setupVisionConstants();

    //get initial evaluation
    step(-1, false, m_ground_truth_training, m_training_performance_log, m_train_image_name);
    step(-1, false, m_ground_truth_test, m_test_performance_log, m_test_image_name);

    while(iteration < total_iterations && !m_halted && err_code == 0) {
        //update gui
        ui->progressBar_opt->setValue(iteration);
        QApplication::processEvents();

        //run batch
        bool training = (iteration%2==0);
        err_code = step(iteration/2, training,
                        (training ? m_ground_truth_training : m_ground_truth_test),
                        (training ? m_training_performance_log : m_test_performance_log),
                        (training ? m_train_image_name : m_test_image_name));   //modulo 2 gives alternating training and testing runs

        iteration++;
    }
    if(!m_halted && err_code==0) {
        //record results
#ifdef MULTI_OPT
        for(int i=0; i<=GENERAL_OPT; i++) {
            OPT_ID id = getIDFromInt(i);
            *(m_optimiser_logs[id]) << m_optimisers[id] << endl;
        }
#else
        m_optimiser_log << m_optimiser << endl;
#endif
        ofstream final((directory + string("best.cfg")).c_str());

        //print out the best combined parameter set
#ifdef MULTI_OPT
        if(VisionConstants::setBallParams(Parameter::getAsVector(m_optimisers[BALL_OPT]->getBest())) &&
                VisionConstants::setGoalBeaconParams(Parameter::getAsVector(m_optimisers[GOAL_BEACON_OPT]->getBest())) &&
                VisionConstants::setObstacleParams(Parameter::getAsVector(m_optimisers[OBSTACLE_OPT]->getBest())) &&
                VisionConstants::setLineParams(Parameter::getAsVector(m_optimisers[LINE_OPT]->getBest())) &&
                VisionConstants::setGeneralParams(Parameter::getAsVector(m_optimisers[GENERAL_OPT]->getBest()))) {
            VisionConstants::print(final);
        }
#else
        if(VisionConstants::setAllOptimisable(Parameter::getAsVector(m_optimiser->getBest()))) {
            VisionConstants::print(final);
        }
#endif
        else {
            QMessageBox::warning(this, "Error", "Error setting parameters. VisionOptimiser::run()");
        }
        QMessageBox::information(this, "Complete", "Optimisation completed successfully.");
    }
    else if(!m_halted) {
        QMessageBox::warning(this, "Error", "Vision frame failed");
    }
    else {
        QMessageBox::information(this, "Cancelled", "Optimisation cancelled before completion.");
    }

}

int VisionOptimiser::step(int iteration, bool training,
                          const vector<vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > > >& ground_truth,
                          ostream& performance_log,
                          const string& stream_name)
{
#ifdef MULTI_OPT
    map<OPT_ID, pair<float, int> > batch_errors;
    map<OPT_ID, float> fitnesses;
    //initialise batch errors
    batch_errors[BALL_OPT] = pair<float, int>(0,0);
    batch_errors[GOAL_BEACON_OPT] = pair<float, int>(0,0);
    batch_errors[OBSTACLE_OPT] = pair<float, int>(0,0);
    batch_errors[LINE_OPT] = pair<float, int>(0,0);
    batch_errors[GENERAL_OPT] = pair<float, int>(0,0);
#else
    pair<float, int> batch_error;
    float fitness;
#endif
    unsigned int frame_no = 0;
    int vision_code;
    //conditional references (differ between training and testing
    //vector<vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > > >& ground_truth = (training ? m_ground_truth_training : m_ground_truth_test);
    //ostream& performance_log = (training ? m_training_performance_log : m_test_performance_log)
    //string& stream_name = (training ? m_train_image_name : m_test_image_name);


    //init gui
    ui->progressBar_strm->setMaximum(ground_truth.size());
    //initialise vision stream
    vision->setImageStream(stream_name);

    //get new params
    if(training) {
#ifdef MULTI_OPT
        VisionConstants::setBallParams(m_optimisers[BALL_OPT]->getNextParameters());
        VisionConstants::setGoalBeaconParams(m_optimisers[GOAL_BEACON_OPT]->getNextParameters());
        VisionConstants::setObstacleParams(m_optimisers[OBSTACLE_OPT]->getNextParameters());
        VisionConstants::setLineParams(m_optimisers[LINE_OPT]->getNextParameters());
        VisionConstants::setGeneralParams(m_optimisers[GENERAL_OPT]->getNextParameters());
#else
        VisionConstants::setAllOptimisable(m_optimiser->getNextParameters());
#endif
    }

    vision_code = vision->runFrame();
    while(vision_code == 0 && frame_no < ground_truth.size()-1 && !m_halted) {
#ifdef MULTI_OPT
        map<OPT_ID, pair<float, int> > frame_error_sums;
        frame_error_sums[BALL_OPT] = pair<float, int>(0,0);
        frame_error_sums[GOAL_BEACON_OPT] = pair<float, int>(0,0);
        frame_error_sums[OBSTACLE_OPT] = pair<float, int>(0,0);
        frame_error_sums[LINE_OPT] = pair<float, int>(0,0);
        frame_error_sums[GENERAL_OPT] = pair<float, int>(0,0);
#else
        pair<float, int> frame_error_sum(0,0);
#endif

        //get errors
        map<VisionFieldObject::VFO_ID, pair<float, int> > frame_errors = vision->evaluateFrame(ground_truth[frame_no], m_false_positive_costs, m_false_negative_costs);

        //debugging
//        for(int i=0; i<VisionFieldObject::INVALID; i++) {
//            VisionFieldObject::VFO_ID vfo_id = VisionFieldObject::getVFOFromNum(i);
//            frame_error += frame_errors.at(vfo_id);
//            cout << VisionFieldObject::getVFOName(vfo_id) << " " << frame_errors[vfo_id].first << " " << frame_errors[vfo_id].second << endl;
//        }

        //accumulate errors
        for(int i=0; i<VisionFieldObject::INVALID; i++) {
            VisionFieldObject::VFO_ID vfo_id = VisionFieldObject::getVFOFromNum(i);
#ifdef MULTI_OPT
            vector<OPT_ID>::const_iterator it;
            for(it = m_vfo_optimiser_map.at(vfo_id).begin(); it != m_vfo_optimiser_map.at(vfo_id).end(); it++) {
                frame_error_sums.at(*it).first += frame_errors.at(vfo_id).first;
                frame_error_sums.at(*it).second += frame_errors.at(vfo_id).second;
            }
#else
            frame_error_sum.first += frame_errors.at(vfo_id).first;
            frame_error_sum.second += frame_errors.at(vfo_id).second;
#endif
        }

#ifdef MULTI_OPT
        for(int i=0; i<=GENERAL_OPT; i++) {
            batch_errors.at(getIDFromInt(i)).first += frame_error_sums.at(getIDFromInt(i)).first;
            batch_errors.at(getIDFromInt(i)).second += frame_error_sums.at(getIDFromInt(i)).second;
        }
#else
        batch_error.first += frame_error_sum.first;
        batch_error.second += frame_error_sum.second;
#endif

        //update gui
        ui->progressBar_strm->setValue(frame_no);
        QApplication::processEvents();

        //next step
        vision_code = vision->runFrame();
        frame_no++;
    }

    if(vision_code == 0 && !m_halted) {
        //update optimiser
#ifdef MULTI_OPT
        for(int i=0; i<=GENERAL_OPT; i++) {
            OPT_ID id = getIDFromInt(i);
            if(batch_errors[id].first == 0)
                fitnesses[id] = numeric_limits<float>::max(); //not likely
            else
                fitnesses[id] = batch_errors[id].second/batch_errors[id].first;
            if(training) {
                m_optimisers[id]->setParametersResult(fitnesses[id]);
            }
        }
#else
        if(batch_error.second == 0)
            fitness = numeric_limits<float>::max(); //not likely
        else
            fitness = batch_error.second/batch_error.first;
        if(training) {
            m_optimiser->setParametersResult(fitness);
        }
#endif

        //write results
        if(training) {
#ifdef MULTI_OPT
            *(m_individual_progress_logs[BALL_OPT]) << VisionConstants::getBallParams() << endl;
            *(m_individual_progress_logs[GOAL_BEACON_OPT]) << VisionConstants::getGoalBeaconParams() << endl;
            *(m_individual_progress_logs[OBSTACLE_OPT]) << VisionConstants::getObstacleParams() << endl;
            *(m_individual_progress_logs[LINE_OPT]) << VisionConstants::getLineParams() << endl;
            *(m_individual_progress_logs[GENERAL_OPT]) << VisionConstants::getGeneralParams() << endl;
#else
            m_progress_log << VisionConstants::getAllOptimisable() << endl;
#endif
         }
#ifdef MULTI_OPT
        performance_log << iteration << " " << fitnesses[BALL_OPT]<< " " << fitnesses[GOAL_BEACON_OPT]<< " " << fitnesses[OBSTACLE_OPT] << " " << fitnesses[LINE_OPT] << " " << fitnesses[GENERAL_OPT] << " " << endl;
#else
        performance_log << iteration << " " << fitness << endl;
#endif
    }

    return vision_code;
}

void VisionOptimiser::setupVisionConstants()
{
    VisionConstants::DO_RADIAL_CORRECTION = false;
    //! Goal filtering constants
    VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_GOALS = false;
    VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS = false;
    VisionConstants::THROWOUT_DISTANT_GOALS = false;
    VisionConstants::THROWOUT_INSIGNIFICANT_GOALS = true;
    VisionConstants::THROWOUT_NARROW_GOALS = true;
    VisionConstants::THROWOUT_SHORT_GOALS = true;
    //! Beacon filtering constants
    VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_BEACONS = false;
    VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BEACONS = false;
    VisionConstants::THROWOUT_DISTANT_BEACONS = false;
    VisionConstants::THROWOUT_INSIGNIFICANT_BEACONS = true;
    //! Ball filtering constants
    VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_BALL = false;
    VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL = false;
    VisionConstants::THROWOUT_SMALL_BALLS = true;
    VisionConstants::THROWOUT_INSIGNIFICANT_BALLS = true;
    VisionConstants::THROWOUT_DISTANT_BALLS = false;
    //! ScanLine options
    VisionConstants::HORIZONTAL_SCANLINE_SPACING = 3;
    VisionConstants::VERTICAL_SCANLINE_SPACING = 3;
    VisionConstants::GREEN_HORIZON_SCAN_SPACING = 11;
    //! Split and Merge constants
    VisionConstants::SAM_MAX_POINTS = 1000;
    VisionConstants::SAM_MAX_LINES = 25;
    VisionConstants::SAM_CLEAR_SMALL = true;
    VisionConstants::SAM_CLEAR_DIRTY = true;
}

void VisionOptimiser::setupCosts()
{
    m_false_positive_costs[VisionFieldObject::BALL] = 200;
    m_false_positive_costs[VisionFieldObject::GOAL_Y_L] = 200;
    m_false_positive_costs[VisionFieldObject::GOAL_Y_R] = 200;
    m_false_positive_costs[VisionFieldObject::GOAL_Y_U] = 200;
    m_false_positive_costs[VisionFieldObject::GOAL_B_L] = 200;
    m_false_positive_costs[VisionFieldObject::GOAL_B_R] = 200;
    m_false_positive_costs[VisionFieldObject::GOAL_B_U] = 200;
    m_false_positive_costs[VisionFieldObject::BEACON_Y] = 200;
    m_false_positive_costs[VisionFieldObject::BEACON_B] = 200;
    m_false_positive_costs[VisionFieldObject::BEACON_U] = 200;
    m_false_positive_costs[VisionFieldObject::OBSTACLE] = 200;
    m_false_positive_costs[VisionFieldObject::FIELDLINE] = 200;

    m_false_negative_costs[VisionFieldObject::BALL] = 200;
    m_false_negative_costs[VisionFieldObject::GOAL_Y_L] = 200;
    m_false_negative_costs[VisionFieldObject::GOAL_Y_R] = 200;
    m_false_negative_costs[VisionFieldObject::GOAL_Y_U] = 200;
    m_false_negative_costs[VisionFieldObject::GOAL_B_L] = 200;
    m_false_negative_costs[VisionFieldObject::GOAL_B_R] = 200;
    m_false_negative_costs[VisionFieldObject::GOAL_B_U] = 200;
    m_false_negative_costs[VisionFieldObject::BEACON_Y] = 200;
    m_false_negative_costs[VisionFieldObject::BEACON_B] = 200;
    m_false_negative_costs[VisionFieldObject::BEACON_U] = 200;
#ifdef MULTI_OPT
    m_false_negative_costs[VisionFieldObject::OBSTACLE] = 100;  //diff
    m_false_negative_costs[VisionFieldObject::FIELDLINE] = 20;  //diff
#else
    m_false_negative_costs[VisionFieldObject::OBSTACLE] = 200;  //diff
    m_false_negative_costs[VisionFieldObject::FIELDLINE] = 200;  //diff
#endif
}
