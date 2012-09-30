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
    //read labels
    ifstream label_file((directory + string("opt_labels.strm")).c_str());

    //set up logs
    m_progress_log.open((directory + m_optimiser->getName() + string("_progress.log")).c_str()),
    m_performance_log.open((directory + m_optimiser->getName() + string("_performance.log")).c_str()),
    m_optimiser_log.open((directory + m_optimiser->getName() + string(".log")).c_str());

    if(!vision->readLabels(label_file, m_ground_truth_full)) {
        QMessageBox::warning(this, "Failure", QString("Failed to read label stream: ") + QString((directory + string("opt_label.strm")).c_str()));
        return;
    }

    //init gui
    ui->progressBar_strm->setMaximum(m_ground_truth_full.size());
    ui->progressBar_opt->setMaximum(total_iterations);
    QApplication::processEvents();

    //initialise vision system
    vision->setImageStream(directory+string("image.strm"));
    vision->setLUT(directory+string("default.lut"));

    while(iteration < total_iterations && !m_halted && err_code == 0) {
        //update gui
        ui->progressBar_opt->setValue(iteration);
        QApplication::processEvents();

        //run batch
        err_code = runOptimisationStep(iteration);

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

int VisionOptimiser::runOptimisationStep(int iteration, bool individual)
{
    float FALSE_POS_COST = 200,
          FALSE_NEG_COST = 20,
          frame_error,
          batch_error = 0,
          fitness;
    unsigned int frame_no = 0;
    int vision_code;

    //get new params
    if(individual) {

    }
    else {
        VisionConstants::setAllOptimisable(m_optimiser->getNextParameters());
    }

    vision_code = vision->runFrame();
    while(vision_code == 0 && frame_no < m_ground_truth_full.size() && !m_halted) {
        frame_error = 0;
        //get errors
        map<VisionFieldObject::VFO_ID, float> frame_errors = vision->evaluateFrame(m_ground_truth_full[frame_no], FALSE_POS_COST, FALSE_NEG_COST);
        for(int i=0; i<VisionFieldObject::INVALID; i++) {
            frame_error += frame_errors[VisionFieldObject::getVFOFromNum(i)];
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
            m_performance_log << iteration << " " << fitness << endl;
        }
    }

    return vision_code;
}
