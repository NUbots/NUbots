#include "visioncomparitor.h"
#include "ui_visioncomparitor.h"
#include "Vision/VisionWrapper/visioncontrolwrappertraining.h"
#include "Vision/visionconstants.h"
#include <QMessageBox>
#include <QInputDialog>

VisionComparitor::VisionComparitor(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::VisionComparitor)
{
    ui->setupUi(this);

    QObject::connect(ui->nextPB, SIGNAL(clicked()), this, SLOT(next()));
    QObject::connect(ui->prevPB, SIGNAL(clicked()), this, SLOT(prev()));
    QObject::connect(ui->exitPB, SIGNAL(clicked()), this, SLOT(halt()));
}

VisionComparitor::~VisionComparitor()
{
    delete ui;
}

/** @brief runs the parameter comparison application.
*   @param image_name File name for the image stream.
*   @param lut_name File name for the look up table
*   @param config0 File name for parameter set 0
*   @param config1 File name for parameter set 1
*/
void VisionComparitor::run(string image_name, string lut_name, string config0, string config1)
{
    VisionControlWrapper* vision = VisionControlWrapper::getInstance();
    ifstream image_file(image_name.c_str());
    cv::Mat mat0, mat1, mat_c;
    LookUpTable lut;

    //setup
    ui->label->setText(config0.c_str());
    ui->label_2->setText(config1.c_str());

    bool ok;
    QStringList l;
    l.append("Lines");
    l.append("All Objects");
    //get the optimiser choice from the user
    QString s = QInputDialog::getItem(this, "Select Display Objects", "Select the preferred objects for display", l, 0, false, &ok);
    bool lines_only;
    if(ok) {
        lines_only = (s.compare("Lines") == 0);
    }
    else {
        return; //user hit cancel
    }

    if(!vision->setLUT(lut_name)) {
        QMessageBox::warning(this, "Failure", QString("Failed to load LUT: ") + QString(lut_name.c_str()));
        return;
    }
    lut.loadLUTFromFile(lut_name);

    if(!image_file.is_open()) {
        QMessageBox::warning(this, "Failure", QString("Failed to open image stream: ") + QString(image_name.c_str()));
        return;
    }
    try {
        //attempt to read the image stream into a frame array
        while(image_file.good()) {
            NUImage img;
            image_file >> img;
            m_frames.push_back(img);
            image_file.peek();
        }
    }
    catch(exception e) {
        QMessageBox::warning(this, "Failure", QString("Invalid image stream file: ") + QString(image_name.c_str()));
        return;
    }
    image_file.close();

    if(m_frames.size() == 0) {
        QMessageBox::warning(this, "Failure", QString("No images in image stream: ") + QString(image_name.c_str()));
        return;
    }

    m_halted = false;
    m_frame_no = 0;
    ui->totalLCD->display((int)m_frames.size());

    while(!m_halted) {
        ui->frameLCD->display((int)(m_frame_no+1));
        m_next = m_prev = false;

        //run the vision system with the first parameter set and render the results
        VisionConstants::loadFromFile(config0);
        vision->runFrame(m_frames[m_frame_no]);
        vision->renderFrame(mat0, lines_only);

        //and again with the second
        VisionConstants::loadFromFile(config1);
        vision->runFrame(m_frames[m_frame_no]);
        vision->renderFrame(mat1, lines_only);

        lut.classifyImage(m_frames[m_frame_no], mat_c);

        //display the rendered images
        display(mat0, mat1, mat_c);

        while(!m_halted && !m_next && !m_prev) {
            QApplication::processEvents();
        }

        //catch if the user tries to progress past the start or end of stream
        if(m_next) {
            m_next = false;
            if(m_frame_no == m_frames.size()-1)
                QMessageBox::information(this, "EOS", "End of stream");
            else
                m_frame_no++;
        }
        if(m_prev) {
            m_prev = false;
            if(m_frame_no == 0)
                QMessageBox::information(this, "BOS", "Beginning of stream");
            else
                m_frame_no--;
        }
    }

    close();
}

/** @brief Displays the two rendered images.
*   @param mat0 Mat for parameter set 0
*   @param mat1 Mat for parameter set 1
*/
void VisionComparitor::display(const cv::Mat& mat0, const cv::Mat& mat1, const cv::Mat& mat_c)
{
    //clear old display
    if(!m_scenes.first.items().empty())
        m_scenes.first.removeItem(m_scenes.first.items().first());
    if(!m_scenes.second.items().empty())
        m_scenes.second.removeItem(m_scenes.second.items().first());
    if(!m_class_scene.items().empty())
        m_class_scene.removeItem(m_class_scene.items().first());

    //point QImage objects at the cv::mat data
    QImage qimg0((uchar*) mat0.data, mat0.cols, mat0.rows, mat0.step, QImage::Format_RGB888);
    QImage qimg1((uchar*) mat1.data, mat1.cols, mat1.rows, mat1.step, QImage::Format_RGB888);
    QImage qimg_c((uchar*) mat_c.data, mat_c.cols, mat_c.rows, mat_c.step, QImage::Format_RGB888);

    //flip the rgb channels
    qimg0 = qimg0.rgbSwapped();
    qimg1 = qimg1.rgbSwapped();
    qimg_c = qimg_c.rgbSwapped();

    //set pixmap and scenes up
    m_pixmaps.first.setPixmap(QPixmap::fromImage(qimg0));
    m_pixmaps.second.setPixmap(QPixmap::fromImage(qimg1));
    m_class_pixmap.setPixmap(QPixmap::fromImage(qimg_c));
    m_scenes.first.addItem(&m_pixmaps.first);
    m_scenes.second.addItem(&m_pixmaps.second);
    m_class_scene.addItem(&m_class_pixmap);

    //display the scenes on the UI
    ui->imageView->setScene(&m_scenes.first);
    ui->imageView_2->setScene(&m_scenes.second);
    ui->imageView_3->setScene(&m_class_scene);
}
