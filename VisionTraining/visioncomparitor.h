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
    Ui::VisionComparitor *ui;

    pair<QGraphicsScene,QGraphicsScene> m_scenes;
    pair<QGraphicsPixmapItem,QGraphicsPixmapItem> m_pixmaps;

    unsigned int m_frame_no;
    vector<NUImage> m_frames;
    bool m_halted,
         m_next,
         m_prev;
};

#endif // VISIONCOMPARITOR_H
