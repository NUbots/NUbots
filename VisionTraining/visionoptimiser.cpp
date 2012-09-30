#include "visionoptimiser.h"
#include "ui_visionoptimiser.h"

#include "Tools/Optimisation/EHCLSOptimiser.h"
#include "Tools/Optimisation/PGRLOptimiser.h"
#include "Tools/Optimisation/PSOOptimiser.h"
#include "Vision/visionconstants.h"
#include <QMessageBox>

VisionOptimiser::OPT_ID VisionOptimiser::getChoiceFromQString(QString str)
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

VisionOptimiser::VisionOptimiser(QWidget *parent, OPT_ID id) :
    QMainWindow(parent),
    ui(new Ui::VisionOptimiser)
{
    ui->setupUi(this);
    switch(id) {
    case EHCLS:
        m_optimiser = new EHCLSOptimiser("VisionEHCLS", VisionConstants::getAllOptimisable());
        break;
    case PGRL:
        m_optimiser = new PGRLOptimiser("VisionPGRL", VisionConstants::getAllOptimisable());
        break;
    case PSO:
        m_optimiser = new PSOOptimiser("VisionPSO", VisionConstants::getAllOptimisable());
        break;
    }
    vision = VisionControlWrapper::getInstance();
}

VisionOptimiser::~VisionOptimiser()
{
    delete ui;
    delete m_optimiser;
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
    m_progress_log.open((directory + m_optimiser->getName() + string("_progress.log")).c_str());
    m_optimiser_log.open((directory + m_optimiser->getName() + string(".log")).c_str());
    m_training_performance_log.open((directory + m_optimiser->getName() + string("_training_performance.log")).c_str());
    m_training_performance_log.setf(ios_base::fixed);
    m_test_performance_log.open((directory + m_optimiser->getName() + string("_test_performance.log")).c_str());
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
    ui->progressBar_opt->setMaximum(total_iterations);
    QApplication::processEvents();

    //initialise vision system
    vision->setLUT(directory+string("default.lut"));

    //set the options we need
    setupVisionConstants();

    while(iteration < total_iterations && !m_halted && err_code == 0) {
        //update gui
        ui->progressBar_opt->setValue(iteration);
        QApplication::processEvents();

        //run batch
        if(iteration%2==0)
            err_code = runTrainingStep(iteration);
        else
            err_code = runEvaluationStep(iteration);

        vision->restartStream();

        iteration++;
    }
    if(!m_halted && err_code==0) {
        //record results
        m_optimiser_log << m_optimiser << endl;
        QMessageBox::information(this, "Complete", "Optimisation completed successfully");
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
          FALSE_NEG_COST = 20,
          frame_error,
          batch_error = 0,
          fitness;
    unsigned int frame_no = 1;
    int vision_code;

    //init gui
    ui->progressBar_strm->setMaximum(m_ground_truth_training.size());
    //initialise vision stream
    vision->setImageStream(m_train_image_name);

    //get new params
    if(individual) {

    }
    else {
        VisionConstants::setAllOptimisable(m_optimiser->getNextParameters());
    }

    vision_code = vision->runFrame();
    while(vision_code == 0 && frame_no < m_ground_truth_training.size() && !m_halted) {
        frame_error = 0;
        //get errors
        map<VisionFieldObject::VFO_ID, float> frame_errors = vision->evaluateFrame(m_ground_truth_training[frame_no], FALSE_POS_COST, FALSE_NEG_COST);
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
            m_optimiser->setParametersResult(fitness);

            //write results
            m_progress_log << VisionConstants::getAllOptimisable() << endl;
            m_training_performance_log << iteration << " " << fitness << endl;
        }
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
    VisionConstants::GREEN_HORIZON_SCAN_SPACING = 6;
    //! Split and Merge constants
    VisionConstants::SAM_MAX_POINTS = 1000;
    VisionConstants::SAM_MAX_LINES = 25;
    VisionConstants::SAM_CLEAR_SMALL = true;
    VisionConstants::SAM_CLEAR_DIRTY = true;
}
