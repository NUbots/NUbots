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


using namespace Vision;

namespace Ui {
    class LabelEditor;
}

class LabelEditor : public QMainWindow
{
    Q_OBJECT

public:
    explicit LabelEditor(QWidget *parent = 0);
    ~LabelEditor();
    
    int run(std::string dir, std::string label_name, std::string image_name);

private:
    void display(const NUImage& frame, const LookUpTable& lut);
    void renderFrame(const NUImage& frame, QImage& img, std::vector<VisionFieldObject*>& gt) const;
    void setInts(std::vector< pair<std::string, Vector3<int> > > vals);
    void setDoubles(std::vector<pair<std::string, Vector3<double> > > vals);
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
    Ui::LabelEditor *ui;                        //! @var The GUI pointer.
    std::vector<QLabel*> m_text_labels;              //! @var The dynamic labels.
    std::vector<QSlider*> m_sliders;                 //! @var The dynamic integer sliders.
    std::vector<QSpinBox*> m_i_spins;                //! @var The dynamic integer inputs.
    std::vector<QDoubleSpinBox*> m_d_spins;          //! @var The dynamic decimal inputs.
    std::vector<VisionFieldObject*> m_ground_truth;  //! @var The current labels.
    std::vector< std::vector<VisionFieldObject*> > m_ground_truth_full;   //! @var All labels.
    QGraphicsScene m_plain_scene;               //! @var The scene for displaying the labelled image.
    QGraphicsScene m_classified_scene;          //! @var The scene for displayying the classified image.
    QGraphicsPixmapItem m_plain_pixmap;         //! @var The labelled image pixmap.
    QGraphicsPixmapItem m_classified_pixmap;    //! @var The classified image pixmap.

    unsigned int m_current_object,  //! @var The index of the current label.
                 m_frame_no,        //! @var The index of the current image.
                 m_total_frames;    //! @var The total number of images.

    bool m_halted,          //! @var Whether the user has halted the labelling.
         m_image_updated,   //! @var Whether the image needs to be re-rendered.
         m_next;            //! @var Whether to progress to the next image.
};

#endif // LABELEDITOR_H
