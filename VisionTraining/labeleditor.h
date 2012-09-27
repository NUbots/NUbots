#ifndef LABELEDITOR_H
#define LABELEDITOR_H

#include <QMainWindow>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
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
    void setInts(vector< pair<string, Vector3<int> > > vals);
    void setDoubles(vector<pair<string, Vector3<double> > > vals);
    void updateCombo();
    void updateControls();

private slots:
    void halt() {m_halted=true;}
    void next() {m_next=true;}
    void addObject();
    void removeObject();
    void changeObject(int i);
    void updateCallbackI(int i) {updateValues();}
    void updateCallbackD(double i) {updateValues();}
    void updateValues();
    
private:
    Ui::LabelEditor *ui;
    vector<QLabel*> m_text_labels;
    vector<QSlider*> m_sliders;
    vector<QSpinBox*> m_i_spins;
    vector<QDoubleSpinBox*> m_d_spins;
    vector<VisionFieldObject*> m_ground_truth;
    vector< vector<VisionFieldObject*> > m_ground_truth_full;
    QGraphicsScene m_plain_scene;
    QGraphicsScene m_classified_scene;
    QGraphicsPixmapItem m_plain_pixmap;
    QGraphicsPixmapItem m_classified_pixmap;

    unsigned int m_current_object,
                 m_frame_no,
                 m_total_frames;
    bool m_halted,
         m_image_updated,
         m_next;
};

#endif // LABELEDITOR_H
