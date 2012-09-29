#include "labeleditor.h"
#include "ui_labeleditor.h"
#include <QMessageBox>
#include <QInputDialog>
#include "Infrastructure/NUImage/NUImage.h"
#include "Infrastructure/NUImage/ColorModelConversions.h"
#include "Tools/Math/General.h"

LabelEditor::LabelEditor(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::LabelEditor)
{
    ui->setupUi(this);
    m_halted = m_image_updated = false;
    m_current_object = m_frame_no = m_total_frames = 0;
    
    //generate interface
    int y=40;
    for(int i=0; i<4; i++) {
        QLabel* label = new QLabel(ui->groupBox);
        QSpinBox* ispin = new QSpinBox(ui->groupBox);
        QDoubleSpinBox* dspin = new QDoubleSpinBox(ui->groupBox);
        QSlider* slider = new QSlider(ui->groupBox);
        label->setGeometry(10, y, 50, 20);
        slider->setGeometry(80, y, 160, 20);
        slider->setOrientation(Qt::Horizontal);
        dspin->setGeometry(80, y, 80, 20);
        dspin->setDecimals(3);
        ispin->setGeometry(250, y, 50, 20);
        label->hide();
        slider->hide();
        dspin->hide();
        ispin->hide();
        m_text_labels.push_back(label);
        m_i_spins.push_back(ispin);
        m_d_spins.push_back(dspin);
        m_sliders.push_back(slider);
        QObject::connect(slider, SIGNAL(valueChanged(int)), ispin, SLOT(setValue(int)));
        QObject::connect(ispin, SIGNAL(valueChanged(int)), slider, SLOT(setValue(int)));
        QObject::connect(slider, SIGNAL(sliderMoved(int)), this, SLOT(updateCallbackI(int)));
        QObject::connect(ispin, SIGNAL(valueChanged(int)), this, SLOT(updateCallbackI(int)));
        QObject::connect(dspin, SIGNAL(valueChanged(double)), this, SLOT(updateCallbackD(double)));
        y+=40;
    }

    //hack
    m_d_spins.at(0)->setSingleStep(1);
    m_d_spins.at(1)->setSingleStep(0.01);
    
    QObject::connect(ui->exitPB, SIGNAL(clicked()), this, SLOT(halt()));
    QObject::connect(ui->nextPB, SIGNAL(clicked()), this, SLOT(next()));
    QObject::connect(ui->addPB, SIGNAL(clicked()), this, SLOT(addObject()));
    QObject::connect(ui->removePB, SIGNAL(clicked()), this, SLOT(removeObject()));
    QObject::connect(ui->comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(changeObject(int)));
}

LabelEditor::~LabelEditor()
{
    delete ui;
}

int LabelEditor::run(string dir)
{
    VisionControlWrapper* vision = VisionControlWrapper::getInstance();
    NUImage frame;
    ifstream label_file((dir + string("labels.strm")).c_str()),
            image_file((dir + string("image.strm")).c_str());
    ofstream label_out((dir + string("opt_labels.strm")).c_str());
    LookUpTable lut;
    lut.loadLUTFromFile(dir + string("default.lut"));

    if(!vision->readLabels(label_file, m_ground_truth_full)) {
        QMessageBox::warning(this, "Failed read!", QString("Failed to read ") + QString(dir.c_str()) + QString("labels.strm"));
        return -1;  //code for failed label read
    }
    m_total_frames = m_ground_truth_full.size();
    m_frame_no = 0;

    while(!m_halted && m_frame_no < m_total_frames) {
        m_next = false;
        m_image_updated = true;
        m_ground_truth = m_ground_truth_full.at(m_frame_no);
        m_current_object = 0;
        updateCombo();
        updateControls();
        image_file >> frame;
        while(!m_halted && !m_next) {
            QApplication::processEvents();
            if(m_image_updated) {
                display(frame, lut);
                m_image_updated = false;
            }
        }

        //write out
        if(!m_halted) {
            label_out << m_ground_truth.size() << endl;
            for(unsigned int i=0; i<m_ground_truth.size(); i++) {
                m_ground_truth.at(i)->printLabel(label_out);
                label_out << endl;
            }
        }
        m_frame_no++;
    }
    if(!m_halted)
        QMessageBox::information(this, "Finished", "End of Stream reached, exitting label editor");
    label_file.close();
    label_out.close();

    close();
    return 0;
}

void LabelEditor::display(const NUImage& frame, const LookUpTable& lut)
{
    cv::Mat plain(frame.getHeight(), frame.getWidth(), CV_8UC3),
            classified(frame.getHeight(), frame.getWidth(), CV_8UC3);
    renderFrame(frame, plain);
    lut.classifyImage(frame, classified);

    //display to actual frames
    //clear old display
    if(!m_plain_scene.items().empty())
        m_plain_scene.removeItem(m_plain_scene.items().first());
    if(!m_classified_scene.items().empty())
        m_classified_scene.removeItem(m_classified_scene.items().first());

    QImage q_plain((uchar*) plain.data, plain.cols, plain.rows, plain.step, QImage::Format_RGB888);
    QImage q_classified((uchar*) classified.data, classified.cols, classified.rows, classified.step, QImage::Format_RGB888);

    q_plain = q_plain.rgbSwapped();
    q_classified = q_classified.rgbSwapped();

    m_plain_pixmap.setPixmap(QPixmap::fromImage(q_plain));
    m_classified_pixmap.setPixmap(QPixmap::fromImage(q_classified));
    m_plain_scene.addItem(&m_plain_pixmap);
    m_classified_scene.addItem(&m_classified_pixmap);

    ui->imageView->setScene(&m_plain_scene);
    ui->imageView_2->setScene(&m_classified_scene);
}

void LabelEditor::renderFrame(const NUImage& frame, cv::Mat& mat)
{
    unsigned char* ptr,
            r, g, b;
    vector<VisionFieldObject*>::const_iterator it;

    for(int y=0; y<frame.getHeight(); y++) {
        ptr = mat.ptr<unsigned char>(y);
        for(int x=0; x<frame.getWidth(); x++) {
            ColorModelConversions::fromYCbCrToRGB(frame(x,y).y, frame(x,y).cb, frame(x,y).cr, r, g, b);
            ptr[3*x]   = b;
            ptr[3*x+1] = g;
            ptr[3*x+2] = r;
        }
    }

    for(it=m_ground_truth.begin(); it<m_ground_truth.end(); it++)
        (*it)->render(mat);
}

void LabelEditor::addObject()
{
    QStringList string_list;
    VisionFieldObject::VFO_ID new_id;
    VisionFieldObject* new_object = NULL;
    bool ok;
    for(int i=0; i<VisionFieldObject::INVALID; i++) {
        VisionFieldObject::VFO_ID id = VisionFieldObject::getVFOFromNum(i);
        string_list.append(QString(VisionFieldObject::getVFOName(id).c_str()));
    }
    QString new_object_name = QInputDialog::getItem(this, "New object", "Select object to add", string_list, 0, false, &ok);

    if(ok) {
        new_id = VisionFieldObject::getVFOFromName(new_object_name.toStdString());
        if(VisionFieldObject::isGoal(new_id)) {
            new_object = dynamic_cast<VisionFieldObject*>(new Goal(new_id, Quad(150,150,165,200)));
        }
        else if(VisionFieldObject::isBeacon(new_id)) {
            new_object = dynamic_cast<VisionFieldObject*>(new Beacon(new_id, Quad(150,150,165,200)));
        }
        else if(new_id == VisionFieldObject::BALL) {
            new_object = dynamic_cast<VisionFieldObject*>(new Ball(PointType(150,150), 15));
        }
        else if(new_id == VisionFieldObject::OBSTACLE) {
            new_object = dynamic_cast<VisionFieldObject*>(new Obstacle(PointType(150,150), 15, 30));
        }
        else if(new_id == VisionFieldObject::FIELDLINE) {
            new_object = dynamic_cast<VisionFieldObject*>(new FieldLine(150, 0));
        }

        if(new_object != NULL) {
            m_ground_truth.push_back(new_object);
            m_current_object = m_ground_truth.size()-1;
            updateCombo();
            updateControls();
            m_image_updated = true;
        }
        else {
            QMessageBox::warning(this, "Invalid object!", QString("Failed to add ") + new_object_name);
        }
    }
}

void LabelEditor::removeObject()
{
    if(m_current_object < m_ground_truth.size()) {
        delete m_ground_truth.at(m_current_object);
        m_ground_truth.erase(m_ground_truth.begin() + m_current_object);
        m_current_object--;
        if(m_current_object > m_ground_truth.size())
            m_current_object = 0;
    }
    else if(!m_ground_truth.empty()){
        QMessageBox::warning(this, "Internal Error!", "Out of bounds access in removeObject()");
    }
    updateCombo();
    updateControls();
    m_image_updated = true;
}

void LabelEditor::changeObject(int i)
{
    if(i >= 0 && i<m_ground_truth.size()) {
        m_current_object = i;
        updateControls();
        m_image_updated = true;
    }
}

void LabelEditor::updateCombo() {
    int cur = m_current_object;
    ui->comboBox->clear();
    for(unsigned int i=0; i<m_ground_truth.size(); i++) {
        cout << m_ground_truth.at(i)->getName() << endl;
        ui->comboBox->addItem(QString(m_ground_truth.at(i)->getName().c_str()));
    }
    m_current_object = cur;
    ui->comboBox->setCurrentIndex(m_current_object);
}

void LabelEditor::updateControls()
{
    if(!m_ground_truth.empty()) {
        VisionFieldObject* vfo = m_ground_truth.at(m_current_object);
        vector< pair<string, Vector3<int> > > vi;
        pair<string, Vector3<int> > pi;
        vector< pair<string, Vector3<double> > > vd;
        pair<string, Vector3<double> > pd;
        VisionFieldObject::VFO_ID id = vfo->getID();
        if(VisionFieldObject::isGoal(id) || VisionFieldObject::isBeacon(id) || id==VisionFieldObject::OBSTACLE) {
            pi.first = "x";
            pi.second = Vector3<int>(vfo->getLocationPixels().x, 0, 319);
            vi.push_back(pi);
            pi.first = "y";
            pi.second = Vector3<int>(vfo->getLocationPixels().y, 0, 239);
            vi.push_back(pi);
            pi.first = "width";
            pi.second = Vector3<int>(vfo->getScreenSize().x, 1, 320);
            vi.push_back(pi);
            pi.first = "height";
            pi.second = Vector3<int>(vfo->getScreenSize().y, 1, 240);
            vi.push_back(pi);
            setInts(vi);
        }
        else if(id==VisionFieldObject::BALL) {
            pi.first = "x";
            pi.second = Vector3<int>(vfo->getLocationPixels().x, -50, 450);
            vi.push_back(pi);
            pi.first = "y";
            pi.second = Vector3<int>(vfo->getLocationPixels().y, -50, 300);
            vi.push_back(pi);
            pi.first = "diameter";
            pi.second = Vector3<int>(dynamic_cast<Ball*>(vfo)->m_diameter, 1, 320);
            vi.push_back(pi);
            setInts(vi);
        }
        else if(id==VisionFieldObject::FIELDLINE) {
            pd.first = "rho";
            pd.second = Vector3<double>(dynamic_cast<FieldLine*>(vfo)->m_rho, -sqrt(319*319+239*239), sqrt(319*319+239*239));
            vd.push_back(pd);
            pd.first = "phi";
            pd.second = Vector3<double>(dynamic_cast<FieldLine*>(vfo)->m_phi, -mathGeneral::PI, mathGeneral::PI);
            vd.push_back(pd);
            setDoubles(vd);
        }
    }
    else {
        for(int i=0; i<m_text_labels.size(); i++) {
            m_text_labels[i]->hide();
            m_sliders[i]->hide();
            m_i_spins[i]->hide();
            m_d_spins[i]->hide();
        }
    }
}

void LabelEditor::setInts(vector< pair<string, Vector3<int> > > vals)
{
    for(int i=0; i<m_text_labels.size(); i++) {
        m_text_labels[i]->hide();
        m_sliders[i]->hide();
        m_i_spins[i]->hide();
        m_d_spins[i]->hide();
    }
    for(unsigned int i=0; i<vals.size(); i++) {
        m_text_labels[i]->show();
        m_text_labels[i]->setText(vals[i].first.c_str());
        m_sliders[i]->show();
        m_sliders[i]->setMinimum(vals[i].second.y);
        m_sliders[i]->setMaximum(vals[i].second.z);
        m_sliders[i]->setValue(vals[i].second.x);
        m_i_spins[i]->show();
        m_i_spins[i]->setMinimum(vals[i].second.y);
        m_i_spins[i]->setMaximum(vals[i].second.z);
        m_i_spins[i]->setValue(vals[i].second.x);
    }

}

void LabelEditor::setDoubles(vector<pair<string, Vector3<double> > > vals)
{
    for(int i=0; i<m_text_labels.size(); i++) {
        m_text_labels[i]->hide();
        m_sliders[i]->hide();
        m_i_spins[i]->hide();
        m_d_spins[i]->hide();
    }
    for(unsigned int i=0; i<vals.size(); i++) {
        m_text_labels[i]->show();
        m_text_labels[i]->setText(vals[i].first.c_str());
        m_d_spins[i]->show();
        m_d_spins[i]->setMinimum(vals[i].second.y);
        m_d_spins[i]->setMaximum(vals[i].second.z);
        m_d_spins[i]->setValue(vals[i].second.x);
    }
}

void LabelEditor::updateValues()
{
    if(!m_ground_truth.empty()) {
        VisionFieldObject* vfo = m_ground_truth.at(m_current_object);
        VisionFieldObject::VFO_ID id = vfo->getID();
        if(VisionFieldObject::isGoal(id) || VisionFieldObject::isBeacon(id) || id==VisionFieldObject::OBSTACLE) {
            vfo->m_location_pixels.x = m_i_spins[0]->value();
            vfo->m_location_pixels.y = m_i_spins[1]->value();
            vfo->m_size_on_screen.x = m_i_spins[2]->value();
            vfo->m_size_on_screen.y = m_i_spins[3]->value();
        }
        else if(id==VisionFieldObject::BALL) {
            vfo->m_location_pixels.x = m_i_spins[0]->value();
            vfo->m_location_pixels.y = m_i_spins[1]->value();
            dynamic_cast<Ball*>(vfo)->m_diameter = m_i_spins[2]->value();
        }
        else if(id==VisionFieldObject::FIELDLINE) {
            dynamic_cast<FieldLine*>(vfo)->m_rho = m_d_spins[0]->value();
            dynamic_cast<FieldLine*>(vfo)->m_phi = m_d_spins[1]->value();
        }
        m_image_updated = true;
    }
}
