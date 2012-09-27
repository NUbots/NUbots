#ifndef VISIONOPTIMISER_H
#define VISIONOPTIMISER_H

#include <QMainWindow>
#include <Tools/Optimisation/Optimiser.h>

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
    
private:
    Ui::VisionOptimiser *ui;
    Optimiser* m_optimiser;
};

#endif // VISIONOPTIMISER_H
