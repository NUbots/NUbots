#ifndef VISIONCOMPARITOR_H
#define VISIONCOMPARITOR_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include "Infrastructure/NUImage/NUImage.h"
#include "opencv2/core/core.hpp"

namespace Ui {
class VisionComparitor;
}

class VisionComparitor : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit VisionComparitor(QWidget *parent = 0);
    ~VisionComparitor();

    void run(string image_name, string lut_name, string config0, string config1);

private:
    void display(const cv::Mat& mat0, const cv::Mat& mat1);

private slots:
    void halt() {m_halted=true;}
    void next() {m_next=true;}
    void prev() {m_prev=true;}

private:
    Ui::VisionComparitor *ui;   //! @var User interface pointer

    pair<QGraphicsScene,QGraphicsScene> m_scenes;               //! @var The two scenes
    pair<QGraphicsPixmapItem,QGraphicsPixmapItem> m_pixmaps;    //! @var The two pixmaps

    unsigned int m_frame_no;    //! @var The current frame index
    vector<NUImage> m_frames;   //! @var The frames read in from file
    bool m_halted,              //! @var Flag for user selecting to exit
         m_next,                //! @var Flag for user selecting to go to next frame
         m_prev;                //! @var Flag for user selecting to go to prev frame
};

#endif // VISIONCOMPARITOR_H
