#include "visionoptimiser.h"
#include "ui_visionoptimiser.h"

#include "Tools/Optimisation/EHCLSOptimiser.h"
#include "Tools/Optimisation/PGRLOptimiser.h"
#include "Tools/Optimisation/PSOOptimiser.h"
#include "Vision/visionconstants.h"

VisionOptimiser::VisionOptimiser(OPT_ID id, QWidget *parent) :
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
}

VisionOptimiser::~VisionOptimiser()
{
    delete ui;
}
