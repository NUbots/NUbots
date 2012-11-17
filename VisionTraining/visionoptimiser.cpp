#include "visionoptimiser.h"
#include "ui_visionoptimiser.h"

#include "Tools/Optimisation/EHCLSOptimiser.h"
#include "Tools/Optimisation/PGRLOptimiser.h"
#include "Tools/Optimisation/PSOOptimiser.h"
#include "Tools/Optimisation/PGAOptimiser.h"
#include "Vision/visionconstants.h"
#include <QMessageBox>

/** @brief String to enum converter for the OPT_TYPE enum.
*/
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

/** @brief String to enum converter for the OPT_ID enum.
*/
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

/** @brief Enum to string converter for the OPT_ID enum.
*/
string VisionOptimiser::getIDName(VisionOptimiser::OPT_ID id)
{
    switch(id) {
    case BALL_OPT: return "BALL";
    case GOAL_BEACON_OPT: return "GOAL_BEACON_OPT";
    case OBSTACLE_OPT: return "OBSTACLE_OPT";
    case LINE_OPT: return "LINE_OPT";
    case GENERAL_OPT: return "GENERAL_OPT";
    default: return "INVALID";
    }
}

VisionOptimiser::VisionOptimiser(QWidget* parent, OPT_TYPE id) :
    QMainWindow(parent),
    ui(new Ui::VisionOptimiser)
{
    ui->setupUi(this);
    QObject::connect(ui->haltPB, SIGNAL(clicked()), this, SLOT(halt()));

    setupCosts(); // load the false negative and positive costs for each object type

    //generate the single or multiple optimisers
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

    //generate a map between VFO_IDs and OPT_IDs
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

#ifdef MULTI_OPT
    for(int i=0; i<=GENERAL_OPT; i++) {
        m_best_fitnesses[getIDFromInt(i)] = -numeric_limits<float>::max();
    }
#else
    m_best_fitness = -numeric_limits<float>::max();
#endif
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

/** @brief Calculates fitness, precision and recall for all object types.
*   @param directory Location of image stream, LUT and parameter set.
*   @note This method uses default names for each file.
*/
void VisionOptimiser::errorPandRevaluation(string directory)
{
    vector<vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > > > gt; //ground truth
    map<OPT_ID, float> fitnesses;                                           //result fitnesses
    map<OPT_ID, pair<double, double> > PR;                                  //results P and R
    string image_name = directory + string("/image.strm");                  //image filename
    ifstream label_file((directory + string("/labels.strm")).c_str());      //label file
    map<VisionFieldObject::VFO_ID, float> zero_costs;                       //empty costs map removes consideration of false positives and negatives

    //load parameter file from directory
    VisionConstants::loadFromFile(directory + string("/VisionOptions.cfg"));

    //read in the labels
    if(!vision->readLabels(label_file, gt)) {
        QMessageBox::warning(this, "Failure", QString("Failed to read label stream: ") + QString((directory + string("label.strm")).c_str()));
        return;
    }

    //initialise vision system
    vision->setLUT(directory+string("/default.lut"));

    //set the options we need
    setupVisionConstants();

    //calculate the fitness P and R
    fitnesses = evaluateBatch(gt, image_name, zero_costs, zero_costs);
    PR = evaluateBatchPR(gt, image_name);

    //print to stdout
//    for(int i=0; i<=GENERAL_OPT; i++) {
//        OPT_ID id = getIDFromInt(i);
//        cout << getIDName(id) << " f: " << fitnesses[id] << " P: " << PR[id].first << " R: " << PR[id].second << endl;
//    }

    cout << getIDName(LINE_OPT) << " f: " << fitnesses[LINE_OPT] << " P: " << PR[LINE_OPT].first << " R: " << PR[LINE_OPT].second << endl;
}

/** @brief Runs a grid search over a pair of parameters.
*   @param directory The directory for all input files.
*   @param grids_per_side The number of divisions for the grid search
*/
void VisionOptimiser::gridSearch(string directory, int grids_per_side)
{
    vector<vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > > > gt;
    map<OPT_ID, float> fitnesses;
    string image_name = directory + string("image.strm");
    ifstream train_label_file((directory + string("labels.strm")).c_str());
    ofstream grid_log((directory + string("grid.txt")).c_str());

    //parameters to search over
    Parameter p1("GREEN_HORIZON_MIN_GREEN_PIXELS", 1, 1, 50),
              p2("GREEN_HORIZON_LOWER_THRESHOLD_MULT", 0, 0, 20);

    //read in the labels
    if(!vision->readLabels(train_label_file, gt)) {
        QMessageBox::warning(this, "Failure", QString("Failed to read label stream: ") + QString((directory + string("label.strm")).c_str()));
        return;
    }

    //initialise vision system
    vision->setLUT(directory+string("default.lut"));

    //set the options we need
    setupVisionConstants();

    bool halt = false;
    grid_log << "x : " << p1.name() << " y : " << p2.name() << endl;
    //search over set number of grids
    for(float x = p1.min(); x<p1.max() && !halt; x+= (p1.max() - p1.min())/grids_per_side) {
        for(float y = p2.min(); y<p2.max() && !halt; y+= (p2.max() - p2.min())/grids_per_side) {
            //change parameters
            if(!VisionConstants::setParameter(p1.name(), x)) {
                cout << "error setting p1" << endl;
                halt = true;
                break;
            }
            if(!VisionConstants::setParameter(p2.name(), (unsigned int)y)) {
                cout << "error setting p2" << endl;
                halt = true;
                break;
            }
            fitnesses = evaluateBatch(gt, image_name, m_false_positive_costs, m_false_negative_costs);
            if(!fitnesses.empty()) {
                //log the general fitness
                grid_log << fitnesses[GENERAL_OPT] << " ";
                //print the others to stdout
                cout << fitnesses[OBSTACLE_OPT] << " ";
                cout << fitnesses[BALL_OPT] << " ";
                cout << fitnesses[GOAL_BEACON_OPT] << " ";
                cout << fitnesses[LINE_OPT] << " ";
                cout << fitnesses[GENERAL_OPT] << endl;
            }
            else {
                cout << "failed" << endl;
                break;
            }
        }
        grid_log << endl;
        cout << 100*x/(p1.max()-p1.min()) << "%" << endl;
    }
    cout << "100%" << endl;
}

/** @brief runs the label editor application.
*   @param dir Directory for input and output files
*   @param total_iterations Number of samples to run for.
*/
void VisionOptimiser::run(string directory, int total_iterations)
{
    int iteration = 0;      //sample counter
    bool success = true;    //flag for failures

    //initialise
    m_halted = false;
    //image streams
    m_training_image_name = directory + string("train_image.strm");
    m_test_image_name = directory + string("test_image.strm");

    //read labels
    ifstream train_label_file((directory + string("train_labels.strm")).c_str());
    ifstream test_label_file((directory + string("test_labels.strm")).c_str());

    total_iterations*=2; //double since we are running once each for training and testing

    //set up logs
    openLogFiles(directory);

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
    printResults(0, evaluateBatch(m_ground_truth_training, m_training_image_name, m_false_positive_costs, m_false_negative_costs), m_training_performance_log);
    printResults(0, evaluateBatch(m_ground_truth_test, m_test_image_name, m_false_positive_costs, m_false_negative_costs), m_test_performance_log);

    iteration = 2;
    while(iteration < total_iterations && !m_halted && success) {
        //update gui
        ui->progressBar_opt->setValue(iteration);
        QApplication::processEvents();

        //run batch
        bool training = (iteration%2==0);   //modulo 2 gives alternating training and testing runs
        if(training) {
            //run a training step - i.e. update the optimisers
            success = trainingStep(iteration/2, m_ground_truth_training, m_training_performance_log, m_training_image_name);
        }
        else {
            //run over the test set, no optimiser update
            printResults(iteration/2, evaluateBatch(m_ground_truth_test, m_test_image_name, m_false_positive_costs, m_false_negative_costs), m_test_performance_log);
        }
        iteration++;
    }
    if(!m_halted && success) {
        //record results
#ifdef MULTI_OPT
        for(int i=0; i<=GENERAL_OPT; i++) {
            OPT_ID id = getIDFromInt(i);
            *(m_optimiser_logs[id]) << m_optimisers[id] << endl;
        }
#else
        m_optimiser_log << m_optimiser << endl;
#endif
        ofstream final((directory + m_opt_name + string("best.cfg")).c_str());

        //print out the best combined parameter set
#ifdef MULTI_OPT
        if(VisionConstants::setBallParams(Parameter::getAsVector(m_best_params[BALL_OPT])) &&
                VisionConstants::setGoalBeaconParams(Parameter::getAsVector(m_best_params[GOAL_BEACON_OPT])) &&
                VisionConstants::setObstacleParams(Parameter::getAsVector(m_best_params[OBSTACLE_OPT])) &&
                VisionConstants::setLineParams(Parameter::getAsVector(m_best_params[LINE_OPT])) &&
                VisionConstants::setGeneralParams(Parameter::getAsVector(m_best_params[GENERAL_OPT]))) {
            VisionConstants::print(final);
        }

#else
        if(VisionConstants::setAllOptimisable(Parameter::getAsVector(m_best_params))) {
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

/** @brief Runs a training step over the batch, updating the optimiser(s).
*   @param iteration The current optimisation sample number.
*   @param ground_truth The labels to use.
*   @param performance_log A log to print the resulting fitness to.
*   @param stream_name The image stream to use.
*/
bool VisionOptimiser::trainingStep(int iteration,
                                   const vector<vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > > >& ground_truth,
                                   ostream& performance_log,
                                   const string& stream_name)
{
    map<OPT_ID, float> fitnesses;   //a map between fitnesses and optimiser ID

    //init gui
    ui->progressBar_strm->setMaximum(ground_truth.size());
    //initialise vision stream
    vision->setImageStream(stream_name);

    //get new params
    bool success = true;
#ifdef MULTI_OPT
    success = success && VisionConstants::setBallParams(m_optimisers[BALL_OPT]->getNextParameters());
    success = success && VisionConstants::setGoalBeaconParams(m_optimisers[GOAL_BEACON_OPT]->getNextParameters());
    success = success && VisionConstants::setObstacleParams(m_optimisers[OBSTACLE_OPT]->getNextParameters());
    success = success && VisionConstants::setLineParams(m_optimisers[LINE_OPT]->getNextParameters());
    success = success && VisionConstants::setGeneralParams(m_optimisers[GENERAL_OPT]->getNextParameters());
#else
    success = success && VisionConstants::setAllOptimisable(m_optimiser->getNextParameters());
#endif
    if(!success) {
        QMessageBox::warning(this, "Error", "Failed to set parameters");
        return false;
    }

    //evaluate the batch
    fitnesses = evaluateBatch(ground_truth, stream_name, m_false_positive_costs, m_false_negative_costs);

    vision->resetHistory();

    if(!fitnesses.empty() && !m_halted) {
        //update optimiser(s)
#ifdef MULTI_OPT
        for(int i=0; i<=GENERAL_OPT; i++) {
            OPT_ID id = getIDFromInt(i);
            m_optimisers[id]->setParametersResult(fitnesses[id]);

            if(fitnesses[id] > m_best_fitnesses[id]) {
                m_best_fitnesses[id] = fitnesses[id];
                m_best_params[id] = getParams(id);
            }
        }
#else
        m_optimiser->setParametersResult(fitnesses[GENERAL_OPT]);
        if(fitnesses[GENERAL_OPT] > m_best_fitness) {
            m_best_fitness = fitnesses[GENERAL_OPT];
            m_best_params = VisionConstants::getAllOptimisable();
        }
#endif

        //write results to logs
#ifdef MULTI_OPT
        *(m_individual_progress_logs[BALL_OPT]) << VisionConstants::getBallParams() << endl;
        *(m_individual_progress_logs[GOAL_BEACON_OPT]) << VisionConstants::getGoalBeaconParams() << endl;
        *(m_individual_progress_logs[OBSTACLE_OPT]) << VisionConstants::getObstacleParams() << endl;
        *(m_individual_progress_logs[LINE_OPT]) << VisionConstants::getLineParams() << endl;
        *(m_individual_progress_logs[GENERAL_OPT]) << VisionConstants::getGeneralParams() << endl;
#else
        m_progress_log << VisionConstants::getAllOptimisable() << endl;
#endif
        printResults(iteration, fitnesses, performance_log);
        return true;
    }
    else {
        return false;
    }
}

/** @brief Evaluates a frame batch for mean fitness.
*   @param ground_truth The labels to use.
*   @param stream_name The image stream to use.
*   @param false_pos_costs A map between field objects and false positive costs.
*   @param false_neg_costs A map between field objects and false negative costs.
*   @return A map of optimiser IDs to fitnesses.
*
*   @note If an empty map for false pos or neg costs is given then no accounting is made for them.
*/
map<VisionOptimiser::OPT_ID, float> VisionOptimiser::evaluateBatch(const vector<vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > > >& ground_truth,
                                                                   const string& stream_name,
                                                                   map<VisionFieldObject::VFO_ID, float>& false_pos_costs,
                                                                   map<VisionFieldObject::VFO_ID, float>& false_neg_costs) const
{
    //initialise batch errors
    map<OPT_ID, float> fitnesses;
    map<OPT_ID, pair<float, int> > batch_errors;
    for(int i=0; i<=GENERAL_OPT; i++)
        batch_errors[getIDFromInt(i)] = pair<float, int>(0,0);
    unsigned int frame_no = 0;
    int vision_code;

    //init gui
    ui->progressBar_strm->setMaximum(ground_truth.size());
    //initialise vision stream
    vision->setImageStream(stream_name);

    vision_code = vision->runFrame();
    while(vision_code == 0 && frame_no < ground_truth.size()-1 && !m_halted) {
        //get errors
        map<VisionFieldObject::VFO_ID, pair<float, int> > frame_errors = vision->evaluateFrame(ground_truth[frame_no], false_pos_costs, false_neg_costs);

        //accumulate errors
        for(int i=0; i<VisionFieldObject::INVALID; i++) {
            VisionFieldObject::VFO_ID vfo_id = VisionFieldObject::getVFOFromNum(i);
            vector<OPT_ID>::const_iterator it;
            //cout << VisionFieldObject::getVFOName(vfo_id) << " " << frame_errors[vfo_id].first << " " << frame_errors[vfo_id].second << endl;
            for(it = m_vfo_optimiser_map.at(vfo_id).begin(); it != m_vfo_optimiser_map.at(vfo_id).end(); it++) {
                batch_errors.at(*it).first += frame_errors.at(vfo_id).first;
                batch_errors.at(*it).second += frame_errors.at(vfo_id).second;
            }
        }

        //update gui
        ui->progressBar_strm->setValue(frame_no);
        QApplication::processEvents();

        //next step
        vision_code = vision->runFrame();
        frame_no++;
    }

    //generate fitnesses from errors
    if(vision_code==0 && !m_halted) {
        for(int i=0; i<=GENERAL_OPT; i++) {
            OPT_ID id = getIDFromInt(i);
            if(batch_errors[id].first == 0) //not likely but just in case
                fitnesses[id] = numeric_limits<float>::max();
            else
                fitnesses[id] = batch_errors[id].second/batch_errors[id].first;
        }
    }
    return fitnesses;
}

/** @brief Evaluates a frame batch for precision and recall.
*   @param ground_truth The labels to use.
*   @param stream_name The image stream to use.
*   @return A map of optimiser IDs to P&R pairs
*/
map<VisionOptimiser::OPT_ID, pair<double, double> > VisionOptimiser::evaluateBatchPR(const vector<vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > > >& ground_truth, const string& stream_name) const
{
    //initialise batch errors
    map<OPT_ID, pair<double, double> > PR;
    map<OPT_ID, Vector3<double> > detection_sum;
    map<VisionFieldObject::VFO_ID, Vector3<double> > detections;
    unsigned int frame_no = 0;
    int vision_code;

    //initialise accumulator
    for(int i=0; i<=GENERAL_OPT; i++)
        detection_sum[getIDFromInt(i)] = Vector3<double>(0,0,0);

    //initialise vision stream
    vision->setImageStream(stream_name);

    vision_code = vision->runFrame();
    while(vision_code == 0 && frame_no < ground_truth.size()-1 && !m_halted) {
        //get errors
        detections = vision->precisionRecall(ground_truth[frame_no]);
        //accumulate errors
        for(int i=0; i<VisionFieldObject::INVALID; i++) {
            VisionFieldObject::VFO_ID vfo_id = VisionFieldObject::getVFOFromNum(i);
            vector<OPT_ID>::const_iterator it;
            for(it = m_vfo_optimiser_map.at(vfo_id).begin(); it != m_vfo_optimiser_map.at(vfo_id).end(); it++) {
                detection_sum.at(*it) += detections.at(vfo_id);
            }
        }

        //next step
        vision_code = vision->runFrame();
        frame_no++;
    }

    for(int i=0; i<=GENERAL_OPT; i++) {
        OPT_ID id = getIDFromInt(i);
        double denom_P = detection_sum.at(id).x + detection_sum.at(id).y,   //TP + FP
               denom_R = detection_sum.at(id).x + detection_sum.at(id).z;   //TP + FN
        if(denom_P != 0)
            PR[id].first = detection_sum.at(id).x/denom_P;    //TP / (TP + FP)
        else
            PR[id].first = 2; //indicates error
        if(denom_R != 0)
            PR[id].second = detection_sum.at(id).x/denom_R;   //TP / (TP + FP)
        else
            PR[id].second = 2; //indicates error
    }

    //generate fitnesses from errors
    return PR;
}

/** @brief Prints fitness results to file.
*   @param iteration The current iteration.
*   @param fitnesses A map between optimiser ID and fitnesses.
*   @param performance_log The stream to print to.
*/
void VisionOptimiser::printResults(int iteration, map<OPT_ID, float> fitnesses, ostream& performance_log) const
{
    if(!fitnesses.empty()) {
        #ifdef MULTI_OPT
            performance_log << iteration << " " << fitnesses[BALL_OPT]<< " " << fitnesses[GOAL_BEACON_OPT]<< " " << fitnesses[OBSTACLE_OPT] << " " << fitnesses[LINE_OPT] << " " << fitnesses[GENERAL_OPT] << " " << endl;
        #else
            performance_log << iteration << " " << fitnesses[GENERAL_OPT] << endl;
        #endif
    }
}

/** @brief Initialises constants that aren't to be optimised.
*/
void VisionOptimiser::setupVisionConstants()
{
    VisionConstants::DO_RADIAL_CORRECTION = false;
    // Goal filtering constants
    VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_GOALS = false;
    VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS = false;
    VisionConstants::THROWOUT_DISTANT_GOALS = false;
    VisionConstants::THROWOUT_INSIGNIFICANT_GOALS = true;
    VisionConstants::THROWOUT_NARROW_GOALS = true;
    VisionConstants::THROWOUT_SHORT_GOALS = true;
    // Beacon filtering constants
    VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_BEACONS = false;
    VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BEACONS = false;
    VisionConstants::THROWOUT_DISTANT_BEACONS = false;
    VisionConstants::THROWOUT_INSIGNIFICANT_BEACONS = true;
    // Ball filtering constants
    VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_BALL = false;
    VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL = false;
    VisionConstants::THROWOUT_SMALL_BALLS = true;
    VisionConstants::THROWOUT_INSIGNIFICANT_BALLS = true;
    VisionConstants::THROWOUT_DISTANT_BALLS = false;
    // ScanLine options
    VisionConstants::HORIZONTAL_SCANLINE_SPACING = 3;
    VisionConstants::VERTICAL_SCANLINE_SPACING = 3;
    VisionConstants::GREEN_HORIZON_SCAN_SPACING = 11;
    // Split and Merge constants
    VisionConstants::SAM_MAX_LINES = 150;
    VisionConstants::SAM_CLEAR_SMALL = true;
    VisionConstants::SAM_CLEAR_DIRTY = true;
    VisionConstants::LINE_METHOD = VisionConstants::SAM;
}

/** @brief Initialises a map of costs for each field object
*/
void VisionOptimiser::setupCosts()
{
    //at present they are all set the same
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
    m_false_negative_costs[VisionFieldObject::OBSTACLE] = 200;
    m_false_negative_costs[VisionFieldObject::FIELDLINE] = 200;
}

/** @brief Opens and initialises the various log files.
*   @param directory Directory for files.
*/
void VisionOptimiser::openLogFiles(string directory)
{
    //log(s) to save the optimiser state(s)
#ifdef MULTI_OPT
    m_optimiser_logs[BALL_OPT] = new ofstream((directory + m_opt_name + string("_ball.log")).c_str());
    m_optimiser_logs[GOAL_BEACON_OPT] = new ofstream((directory + m_opt_name + string("_goalbeacon.log")).c_str());
    m_optimiser_logs[OBSTACLE_OPT] = new ofstream((directory + m_opt_name + string("_obstacle.log")).c_str());
    m_optimiser_logs[LINE_OPT] = new ofstream((directory + m_opt_name + string("_line.log")).c_str());
    m_optimiser_logs[GENERAL_OPT] = new ofstream((directory + m_opt_name + string("_general.log")).c_str());
#else
    m_optimiser_log.open((directory + m_opt_name + string(".log")).c_str());
#endif

    //log(s) to save the parameter sets trialed
#ifdef MULTI_OPT
    m_individual_progress_logs[BALL_OPT] = new ofstream((directory + m_opt_name + string("_ball_progress.log")).c_str());
    m_individual_progress_logs[GOAL_BEACON_OPT] = new ofstream((directory + m_opt_name + string("_goalbeacon_progress.log")).c_str());
    m_individual_progress_logs[OBSTACLE_OPT] = new ofstream((directory + m_opt_name + string("_obstacle_progress.log")).c_str());
    m_individual_progress_logs[LINE_OPT] = new ofstream((directory + m_opt_name + string("_line_progress.log")).c_str());
    m_individual_progress_logs[GENERAL_OPT] = new ofstream((directory + m_opt_name + string("_general_progress.log")).c_str());
#else
    m_progress_log.open((directory + m_opt_name + string("_progress.log")).c_str());
#endif

    //logs to record the training and test performance
    m_training_performance_log.open((directory + m_opt_name + string("_training_performance.log")).c_str());
    m_training_performance_log.setf(ios_base::fixed);
    m_test_performance_log.open((directory + m_opt_name + string("_test_performance.log")).c_str());
    m_test_performance_log.setf(ios_base::fixed);

    //print headers to the performance logs
#ifdef MULTI_OPT
    m_training_performance_log << "Iteration Ball GoalBeacon Obstacle Line General" << endl;
    m_test_performance_log << "Iteration Ball GoalBeacon Obstacle Line General" << endl;
#else
    m_training_performance_log << "Iteration Fitness" << endl;
    m_test_performance_log << "Iteration Fitness" << endl;
#endif
}

/** @brief Retrieves the parameters associated with the optimiser id
*   @param id The optimiser identifier.
*/
vector<Parameter> VisionOptimiser::getParams(OPT_ID id)
{
    switch(id) {
    case BALL_OPT: return VisionConstants::getBallParams();
    case GOAL_BEACON_OPT: return VisionConstants::getGoalBeaconParams();
    case OBSTACLE_OPT: return VisionConstants::getObstacleParams();
    case LINE_OPT: return VisionConstants::getLineParams();
    case GENERAL_OPT: return VisionConstants::getGeneralParams();
    }
}
