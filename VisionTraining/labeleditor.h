#ifndef LABELEDITOR_H
#define LABELEDITOR_H

#include <QMainWindow>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QSlider>
#include <vector>
#include "Vision/VisionWrapper/visioncontrolwrappertraining.h"

using namespace std;

namespace Ui {
    class LabelEditor;
}

class LabelEditor : public QMainWindow
{
    Q_OBJECT

public:
    explicit LabelEditor(QWidget *parent = 0);
    ~LabelEditor();
    
    int run(string dir);

private:
    void display(const NUImage& frame, const LookUpTable& lut);
    void renderFrame(const NUImage& frame, cv::Mat& mat);
    void setInts(vector< pair<string,int> > vals);
    void setDoubles(vector< pair<string,double> > vals);
    void updateControls();

private slots:
    void halt() {halted=true;}
    void addObject();
    void removeObject();
    void changeObject(int i);
    void updateValues();
    
private:
    Ui::LabelEditor *ui;
    vector<QLabel*> m_text_labels;
    vector<QSlider*> m_sliders;
    vector<QSpinBox*> m_i_spins;
    vector<QDoubleSpinBox*> m_d_spins;
    vector<VisionFieldObject*> m_ground_truth;
    unsigned int m_current_object;
    bool halted;
};

#endif // LABELEDITOR_H
