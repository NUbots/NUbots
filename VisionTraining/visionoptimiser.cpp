#include "visionoptimiser.h"
#include "ui_visionoptimiser.h"

#include "Tools/Optimisation/EHCLSOptimiser.h"
#include "Tools/Optimisation/PGRLOptimiser.h"
#include "Tools/Optimisation/PSOOptimiser.h"
#include "Vision/visionconstants.h"
#include <QMessageBox>

VisionOptimiser::OPT_TYPE VisionOptimiser::getChoiceFromQString(QString str)
{
    if(str.compare("PGRL") == 0) {
        return PGRL;
    }
    else if(str.compare("EHCLS") == 0) {
        return EHCLS;
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

VisionOptimiser::VisionOptimiser(QWidget* parent, bool individual, OPT_TYPE id) :
    QMainWindow(parent),
    ui(new Ui::VisionOptimiser)
{
    ui->setupUi(this);
    switch(id) {
    case EHCLS:
        opt_name = "VisionEHCLS";
        m_optimisers[BALL_OPT] = new EHCLSOptimiser("EHCLSBall", VisionConstants::getBallParams());
        m_optimisers[GOAL_BEACON_OPT] = new EHCLSOptimiser("EHCLSGoalBeacon", VisionConstants::getGoalBeaconParams());
        m_optimisers[OBSTACLE_OPT] = new EHCLSOptimiser("EHCLSObstacle", VisionConstants::getObstacleParams());
        m_optimisers[LINE_OPT] = new EHCLSOptimiser("EHCLSLine", VisionConstants::getLineParams());
        m_optimisers[GENERAL_OPT] = new EHCLSOptimiser("EHCLSGeneral", VisionConstants::getGeneralParams());
        //m_optimiser = new EHCLSOptimiser("VisionEHCLS", VisionConstants::getAllOptimisable());
        break;
    case PGRL:
        opt_name = "VisionPGRL";
        m_optimisers[BALL_OPT] = new PGRLOptimiser("PGRLBall", VisionConstants::getBallParams());
        m_optimisers[GOAL_BEACON_OPT] = new PGRLOptimiser("PGRLGoalBeacon", VisionConstants::getGoalBeaconParams());
        m_optimisers[OBSTACLE_OPT] = new PGRLOptimiser("PGRLObstacle", VisionConstants::getObstacleParams());
        m_optimisers[LINE_OPT] = new PGRLOptimiser("PGRLLine", VisionConstants::getLineParams());
        m_optimisers[GENERAL_OPT] = new PGRLOptimiser("PGRLGeneral", VisionConstants::getGeneralParams());
        //m_optimiser = new PGRLOptimiser("VisionPGRL", VisionConstants::getAllOptimisable());
        break;
    case PSO:
        opt_name = "VisionPSO";
        m_optimisers[BALL_OPT] = new PSOOptimiser("PSOBall", VisionConstants::getBallParams());
        m_optimisers[GOAL_BEACON_OPT] = new PSOOptimiser("PSOGoalBeacon", VisionConstants::getGoalBeaconParams());
        m_optimisers[OBSTACLE_OPT] = new PSOOptimiser("PSOObstacle", VisionConstants::getObstacleParams());
        m_optimisers[LINE_OPT] = new PSOOptimiser("PSOLine", VisionConstants::getLineParams());
        m_optimisers[GENERAL_OPT] = new PSOOptimiser("PSOGeneral", VisionConstants::getGeneralParams());
        //m_optimiser = new PSOOptimiser("VisionPSO", VisionConstants::getAllOptimisable());
        break;
    }

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

    vision = VisionControlWrapper::getInstance();
}

VisionOptimiser::~VisionOptimiser()
{
    delete ui;
    //delete m_optimiser;
    delete m_optimisers[BALL_OPT];
    delete m_optimisers[GOAL_BEACON_OPT];
    delete m_optimisers[OBSTACLE_OPT];
    delete m_optimisers[LINE_OPT];
    delete m_optimisers[GENERAL_OPT];
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

    //set up logs
//    m_progress_log.open((directory + m_optimiser->getName() + string("_progress.log")).c_str());
//    m_optimiser_log.open((directory + m_optimiser->getName() + string(".log")).c_str());
//    m_training_performance_log.open((directory + m_optimiser->getName() + string("_training_performance.log")).c_str());
//    m_training_performance_log.setf(ios_base::fixed);
//    m_test_performance_log.open((directory + m_optimiser->getName() + string("_test_performance.log")).c_str());
//    m_test_performance_log.setf(ios_base::fixed);
    m_progress_log.open((directory + opt_name + string("_progress.log")).c_str());
    //m_optimiser_log.open((directory + opt_name + string(".log")).c_str());
    m_optimiser_logs[BALL_OPT] = new ofstream((directory + opt_name + string("_ball.log")).c_str());
    m_optimiser_logs[GOAL_BEACON_OPT] = new ofstream((directory + opt_name + string("_goalbeacon.log")).c_str());
    m_optimiser_logs[OBSTACLE_OPT] = new ofstream((directory + opt_name + string("_obstacle.log")).c_str());
    m_optimiser_logs[LINE_OPT] = new ofstream((directory + opt_name + string("_line.log")).c_str());
    m_optimiser_logs[GENERAL_OPT] = new ofstream((directory + opt_name + string("_general.log")).c_str());

    m_individual_progress_logs[BALL_OPT] = new ofstream((directory + opt_name + string("_ball_progress.log")).c_str());
    m_individual_progress_logs[GOAL_BEACON_OPT] = new ofstream((directory + opt_name + string("_goalbeacon_progress.log")).c_str());
    m_individual_progress_logs[OBSTACLE_OPT] = new ofstream((directory + opt_name + string("_obstacle_progress.log")).c_str());
    m_individual_progress_logs[LINE_OPT] = new ofstream((directory + opt_name + string("_line_progress.log")).c_str());
    m_individual_progress_logs[GENERAL_OPT] = new ofstream((directory + opt_name + string("_general_progress.log")).c_str());

    m_training_performance_log.open((directory + opt_name + string("_training_performance.log")).c_str());
    m_training_performance_log.setf(ios_base::fixed);
    m_test_performance_log.open((directory + opt_name + string("_test_performance.log")).c_str());
    m_test_performance_log.setf(ios_base::fixed);

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

    while(iteration < total_iterations*2 && !m_halted && err_code == 0) {
        //update gui
        ui->progressBar_opt->setValue(iteration);
        QApplication::processEvents();

        //run batch
        if(iteration%2==0)
            err_code = runTrainingStep(iteration/2);
        else
            err_code = runEvaluationStep(iteration/2);

        vision->restartStream();

        iteration++;
    }
    if(!m_halted && err_code==0) {
        //record results
        //m_optimiser_log << m_optimiser << endl;
        for(int i=0; i<=GENERAL_OPT; i++) {
            OPT_ID id = getIDFromInt(i);
            *(m_optimiser_logs[id]) << m_optimisers[id] << endl;
        }
        ofstream final((directory + string("best.cfg")).c_str());
//        if(VisionConstants::setAllOptimisable(Parameter::getAsVector(m_optimiser->getBest()))) {
//            VisionConstants::print(final);
//        }
        if(VisionConstants::setBallParams(Parameter::getAsVector(m_optimisers[BALL_OPT]->getBest())) &&
                VisionConstants::setGoalBeaconParams(Parameter::getAsVector(m_optimisers[GOAL_BEACON_OPT]->getBest())) &&
                VisionConstants::setObstacleParams(Parameter::getAsVector(m_optimisers[OBSTACLE_OPT]->getBest())) &&
                VisionConstants::setLineParams(Parameter::getAsVector(m_optimisers[LINE_OPT]->getBest())) &&
                VisionConstants::setGeneralParams(Parameter::getAsVector(m_optimisers[GENERAL_OPT]->getBest()))) {
            VisionConstants::print(final);
        }
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

int VisionOptimiser::runTrainingStep(int iteration, bool individual)
{
    float FALSE_POS_COST = 200,
          FALSE_NEG_COST = 200;
            //batch_error = 0,
            //fitness;
    map<OPT_ID, float> batch_errors,
                       fitnesses;
    batch_errors[BALL_OPT] = 0;
    batch_errors[GOAL_BEACON_OPT] = 0;
    batch_errors[OBSTACLE_OPT] = 0;
    batch_errors[LINE_OPT] = 0;
    batch_errors[GENERAL_OPT] = 0;
    unsigned int frame_no = 0;
    int vision_code;

    //init gui
    ui->progressBar_strm->setMaximum(m_ground_truth_training.size());
    //initialise vision stream
    vision->setImageStream(m_train_image_name);

    //get new params
    VisionConstants::setBallParams(m_optimisers[BALL_OPT]->getNextParameters());
    VisionConstants::setGoalBeaconParams(m_optimisers[GOAL_BEACON_OPT]->getNextParameters());
    VisionConstants::setObstacleParams(m_optimisers[OBSTACLE_OPT]->getNextParameters());
    VisionConstants::setLineParams(m_optimisers[LINE_OPT]->getNextParameters());
    VisionConstants::setGeneralParams(m_optimisers[GENERAL_OPT]->getNextParameters());

    //VisionConstants::setAllOptimisable(m_optimiser->getNextParameters());


    vision_code = vision->runFrame();
    while(vision_code == 0 && frame_no < m_ground_truth_training.size()-1 && !m_halted) {
        map<OPT_ID, float> frame_error_sums;
        frame_error_sums[BALL_OPT] = 0;
        frame_error_sums[GOAL_BEACON_OPT] = 0;
        frame_error_sums[OBSTACLE_OPT] = 0;
        frame_error_sums[LINE_OPT] = 0;
        frame_error_sums[GENERAL_OPT] = 0;
        //frame_error = 0;

        //get errors
        map<VisionFieldObject::VFO_ID, float> frame_errors = vision->evaluateFrame(m_ground_truth_training[frame_no], FALSE_POS_COST, FALSE_NEG_COST);

        //accumulate errors
        for(int i=0; i<VisionFieldObject::INVALID; i++) {
            VisionFieldObject::VFO_ID vfo_id = VisionFieldObject::getVFOFromNum(i);
            //frame_error += frame_errors.at(vfo_id);
            vector<OPT_ID>::const_iterator it;
            for(it = m_vfo_optimiser_map.at(vfo_id).begin(); it != m_vfo_optimiser_map.at(vfo_id).end(); it++) {
                frame_error_sums.at(*it) += frame_errors.at(vfo_id);
            }
        }
//        batch_error += frame_error;
        for(int i=0; i<=GENERAL_OPT; i++) {
            batch_errors.at(getIDFromInt(i)) += frame_error_sums.at(getIDFromInt(i));
        }


        //update gui
        ui->progressBar_strm->setValue(frame_no);
        QApplication::processEvents();

        //next step
        vision_code = vision->runFrame();
        frame_no++;
    }

    if(vision_code == 0 && !m_halted) {
        //update optimiser
//        if(batch_error == 0)
//            fitness = numeric_limits<float>::max(); //not likely
//        else
//            fitness = 1.0/batch_error;
        for(int i=0; i<=GENERAL_OPT; i++) {
            OPT_ID id = getIDFromInt(i);
            if(batch_errors[id] == 0)
                fitnesses[id] = numeric_limits<float>::max(); //not likely
            else
                fitnesses[id] = 1.0/batch_errors[id];
            m_optimisers[id]->setParametersResult(fitnesses[id]);
        }
        //m_optimiser->setParametersResult(fitness);

        //write results
        m_progress_log << VisionConstants::getAllOptimisable() << endl;
        *(m_individual_progress_logs[BALL_OPT]) << VisionConstants::getBallParams() << endl;
        *(m_individual_progress_logs[GOAL_BEACON_OPT]) << VisionConstants::getGoalBeaconParams() << endl;
        *(m_individual_progress_logs[OBSTACLE_OPT]) << VisionConstants::getObstacleParams() << endl;
        *(m_individual_progress_logs[LINE_OPT]) << VisionConstants::getLineParams() << endl;
        *(m_individual_progress_logs[GENERAL_OPT]) << VisionConstants::getGeneralParams() << endl;

        m_training_performance_log << iteration << " " << fitnesses[BALL_OPT]<< " " << fitnesses[GOAL_BEACON_OPT]<< " " << fitnesses[OBSTACLE_OPT] << " " << fitnesses[LINE_OPT] << " " << fitnesses[GENERAL_OPT] << " " << endl;
    }

    return vision_code;
}

int VisionOptimiser::runEvaluationStep(int iteration, bool individual)
{
    float FALSE_POS_COST = 200,
          FALSE_NEG_COST = 20,
          frame_error,
          batch_error = 0,
          fitness;
    unsigned int frame_no = 1;
    int vision_code;

    //init gui
    ui->progressBar_strm->setMaximum(m_ground_truth_test.size());
    //initialise vision stream
    vision->setImageStream(m_test_image_name);
    //uses last parameters

    vision_code = vision->runFrame();
    while(vision_code == 0 && frame_no < m_ground_truth_test.size() && !m_halted) {
        frame_error = 0;
        //get errors
        map<VisionFieldObject::VFO_ID, float> frame_errors = vision->evaluateFrame(m_ground_truth_test[frame_no], FALSE_POS_COST, FALSE_NEG_COST);
        for(int i=0; i<VisionFieldObject::INVALID; i++) {
            frame_error += frame_errors.at(VisionFieldObject::getVFOFromNum(i));
        }
        batch_error += frame_error;

        //update gui
        ui->progressBar_strm->setValue(frame_no);
        QApplication::processEvents();

        //next step
        vision_code = vision->runFrame();
        frame_no++;
    }

    if(vision_code == 0 && !m_halted) {
        //update optimiser
        if(individual) {

        }
        else {
            if(batch_error == 0)
                fitness = numeric_limits<float>::max(); //not likely
            else
                fitness = 1.0/batch_error;
            //write results
            m_test_performance_log << iteration << " " << fitness << endl;
        }
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
