#include "visioncontrolwrapperqt.h"
#include "nubotdataconfig.h"
#include <QApplication>
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include "startoptionsdialog.h"

// FOR GOAL PAPER CODE
//#include <boost/accumulators/accumulators.hpp>
//#include <boost/accumulators/statistics/stats.hpp>
//#include <boost/accumulators/statistics/mean.hpp>
//#include <boost/accumulators/statistics/variance.hpp>

using namespace boost::accumulators;
//END DEBUG

VisionControlWrapper* VisionControlWrapper::instance = 0;

VisionControlWrapper* VisionControlWrapper::getInstance()
{
    if(!instance)
        instance = new VisionControlWrapper();
    return instance;
}

VisionControlWrapper::VisionControlWrapper()
{
    wrapper = NULL;
}

// GOAL PAPER CODE
//int VisionControlWrapper::run()
//{
//    std::string base = std::string(getenv("HOME")) + "/nubot/Images/GoalPaper/";
//    std::vector<std::string> leans;
//    std::vector<int> dists;
//    leans.push_back("No_Lean"); dists.push_back(60);
//    leans.push_back("No_Lean"); dists.push_back(150);
//    leans.push_back("No_Lean"); dists.push_back(300);
//    leans.push_back("No_Lean"); dists.push_back(350);
//    leans.push_back("No_Lean"); dists.push_back(530);
//    leans.push_back("No_Lean"); dists.push_back(600);

//    leans.push_back("Small_Lean"); dists.push_back(60);
//    leans.push_back("Small_Lean"); dists.push_back(150);
//    leans.push_back("Small_Lean"); dists.push_back(300);
//    leans.push_back("Small_Lean"); dists.push_back(350);
//    leans.push_back("Small_Lean"); dists.push_back(530);
//    leans.push_back("Small_Lean"); dists.push_back(600);

//    leans.push_back("Large_Lean"); dists.push_back(60);
//    leans.push_back("Large_Lean"); dists.push_back(150);
//    leans.push_back("Large_Lean"); dists.push_back(300);
//    leans.push_back("Large_Lean"); dists.push_back(350);
//    leans.push_back("Large_Lean"); dists.push_back(530);
//    leans.push_back("Large_Lean"); dists.push_back(600);

//    std::string cfg = std::string(CONFIG_DIR) + std::string("VisionOptions.cfg"),
//           lname = std::string(DATA_DIR) +  std::string("/default.lut");

//    ofstream h((base + "h.txt").c_str());
//    ofstream r1((base + "r1.txt").c_str());
//    ofstream r2((base + "r2.txt").c_str());

//    gui.show();

//    for(size_t i=0; i<leans.size(); i++) {
//        int frame = 1,
//            error = 0;
//        bool finished = false;
//        if(wrapper)
//            delete wrapper;
//        std::stringstream nm;
//        nm << base << leans[i] << "/" << dists[i] << "/image.strm";
//        wrapper = new DataWrapper(&gui, true, DataWrapper::STREAM, nm.str(), "", cfg, lname);
//        DataWrapper::instance = wrapper;

//        if(dists[i] < 300)
//            wrapper->true_num_posts = 1;
//        else
//            wrapper->true_num_posts = 2;

//        if(leans[i].compare("Small_Lean") == 0) {
//            wrapper->kinematics_horizon.setLineFromPoints(Point(0, 0), Point(319, 320*tan(0.1701313268)));
//        }
//        else if(leans[i].compare("Large_Lean") == 0) {
//            wrapper->kinematics_horizon.setLineFromPoints(Point(0, 0), Point(319, 320*tan(0.4)));
//        }

//        while(error == 0 && !finished) {
//            gui.resetFlags();
//            gui.setFrameNo(frame);
//            error = runFrame();
//            gui.refresh();
//            QApplication::processEvents();
//            while(!gui.next() && !finished && error == 0) {
//                QApplication::processEvents();
//                finished = gui.finished();
//            }
//            QApplication::processEvents();
//            finished = gui.finished();
//            frame++;
//        }
//        if(finished)
//            break;

//        h << leans[i] << " " << dists[i] << " ";
//        r1 << leans[i] << " " << dists[i] << " ";
//        r2 << leans[i] << " " << dists[i] << " ";
//        double rat_h = 1 - wrapper->ratio_hist.first/wrapper->ratio_hist.second,
//                rat_r1 = 1 - wrapper->ratio_r1.first/wrapper->ratio_r1.second,
//                rat_r2 = 1 - wrapper->ratio_r2.first/wrapper->ratio_r2.second;


//        std::cout << wrapper->ratio_hist.first << " " << wrapper->ratio_hist.second << " " << frame << std::endl;
//        std::cout << wrapper->ratio_r1.first << " " << wrapper->ratio_r1.second << " " << frame << std::endl;
//        std::cout << wrapper->ratio_r2.first << " " << wrapper->ratio_r2.second << " " << frame << std::endl;
//        if(rat_h > 0)
//            h << mean(wrapper->acc_hist) << " " << sqrt(variance(wrapper->acc_hist)) << " " << rat_h << std::endl;
//        else
//            h << 0 << " " << 0 << " " << rat_h << std::endl;

//        if(rat_r1 > 0)
//            r1 << mean(wrapper->acc_r1) << " " << sqrt(variance(wrapper->acc_r1)) << " " << rat_r1 << std::endl;
//        else
//            r1 << 0 << " " << 0 << " " << rat_r1 << std::endl;

//        if(rat_r2 > 0)
//            r2 << mean(wrapper->acc_r2) << " " << sqrt(variance(wrapper->acc_r2)) << " " << rat_r2 << std::endl;
//        else
//            r2 << 0 << " " << 0 << " " << rat_r2 << std::endl;
//    }

//    gui.hide();

//    h.close();
//    r1.close();
//    r2.close();

//    return 0;
//}

int VisionControlWrapper::run()
{
    //SETUP DATA WRAPPER
    DataWrapper::INPUT_METHOD method;
    bool ok;
    std::string istrm = "";
    std::string sstrm = "";
    std::string cfg = "";
    std::string lname = "";

    getOptions(method, ok, istrm, sstrm, cfg, lname);

    if(ok) {
        wrapper = new DataWrapper(&gui, ok, method, istrm, sstrm, cfg, lname);
        DataWrapper::instance = wrapper;

        //BEGIN
        int frame = 1;
        int error = 0;
        bool finished = false;
        bool next = true;
        bool prev;

        gui.show();
        while(!finished)
        {
            gui.resetFlags();
            gui.setFrameNo(frame);

            // don't run if prev was cancelled due to beginning of stream
            if(next)
                error = runFrame(true);
            else if(prev)
                error = runFrame(false);

            next = false;
            prev = false;
            if(error)
            {
                if(QMessageBox::question(&gui, "", "No more frames, reset stream?", QMessageBox::Ok | QMessageBox::Cancel) == QMessageBox::Ok) {
                    delete wrapper;
                    wrapper = new DataWrapper(&gui, ok, method, istrm, sstrm, cfg, lname);
                    DataWrapper::instance = wrapper;
                }
                else {
                    finished = true;
                }
            }
            else {
                gui.refresh();
                while(!next && !prev && !finished && error == 0)
                {
                    QApplication::processEvents();
                    next = gui.next();
                    prev = gui.prev();
                    finished = gui.finished();
                }
                QApplication::processEvents();
                finished = gui.finished();

                // advance
                if(next) {
                    frame++;
                }
                // only move back if we aren't at the beginning
                else if(prev){
                    if(frame == 1)
                        prev = false;
                    else
                        frame--;
                }
            }
        }
        gui.hide();
        return error;
    }

    return 0;
}

int VisionControlWrapper::runFrame(bool forward)
{
    if(!wrapper->updateFrame(forward))
    {
        #if VISION_WRAPPER_VERBOSITY > 1
            debug << "VisionControlWrapper::runFrame() - updateFrame() failed" << std::endl;
        #endif
        return -1;  //failure - do not run vision
    }
    return controller.runFrame(true, true, true, true);
}

void VisionControlWrapper::getOptions(DataWrapper::INPUT_METHOD& method, bool& ok, std::string& istrm, std::string& sstrm, std::string& cfg, std::string& lname)
{
    StartOptionsDialog d;
    bool using_sensors;

    d.exec();

    ok = d.isValid();
    if(ok) {
        if(d.isInputCamera()) {
            method = DataWrapper::CAMERA;
            using_sensors = false;
            if(d.isAutomatic()) {
                lname = string(DATA_DIR) +  string("/default.lut");
                cfg = string(CONFIG_DIR) + string("/VisionOptions.cfg");
            }
            else if(d.isFolderBased()) {
                QString dir = QFileDialog::getExistingDirectory(NULL, "Select Configuration Folder", QString(getenv("HOME")) + QString("/nubot/"));
                ok = !dir.isEmpty();
                if(ok) {
                    lname = dir.toStdString() + string("/default.lut");
                    cfg = dir.toStdString() + string("/VisionOptions.cfg");
                }
            }
            else {
                ok = false;
                lname = QFileDialog::getOpenFileName(NULL, "Select Lookup Table", QString(getenv("HOME")) + QString("/nubot/"),  "LUT Files (*.lut)").toStdString();
                if(!lname.empty()) {
                    cfg = QFileDialog::getOpenFileName(NULL, "Select Configuration File", QString(getenv("HOME")) + QString("/nubot/Config/Darwin/"), "config Files (*.cfg)").toStdString();
                    ok = !cfg.empty();
                }
            }
        }
        else {
            //stream based
            method = DataWrapper::STREAM;
            using_sensors = d.isUsingSensors();

            if(d.isAutomatic()) {
                istrm = string(DATA_DIR) + string("/image.strm");
                if(using_sensors)
                    sstrm = string(DATA_DIR) + string("/sensor.strm");
                lname = std::string(DATA_DIR) +  std::string("/default.lut");
                cfg = std::string(CONFIG_DIR) + std::string("/VisionOptions.cfg");
            }
            else if(d.isFolderBased()) {
                QString dir = QFileDialog::getExistingDirectory(NULL, "Select Configuration Folder", QString(getenv("HOME")) + QString("/nubot/"));
                ok = !dir.isEmpty();
                if(ok) {
                    istrm = dir.toStdString() + string("/image.strm");
                    if(using_sensors)
                        sstrm = dir.toStdString() + string("/sensor.strm");
                    lname = dir.toStdString() + string("/default.lut");
                    cfg = dir.toStdString() + string("/VisionOptions.cfg");
                }
            }
            else {
                ok = false;
                istrm = QFileDialog::getOpenFileName(NULL, "Select image stream", QString(getenv("HOME")) + QString("/nubot/"),  "Stream Files (*.strm)").toStdString();
                if(!istrm.empty()) {
                    lname = QFileDialog::getOpenFileName(NULL, "Select Lookup Table", QString(getenv("HOME")) + QString("/nubot/"),  "LUT Files (*.lut)").toStdString();
                    if(!lname.empty()) {
                        cfg = QFileDialog::getOpenFileName(NULL, "Select Configuration File", QString(getenv("HOME")) + QString("/nubot/Config/Darwin/"), "config Files (*.cfg)").toStdString();
                        if(using_sensors)
                            sstrm = QFileDialog::getOpenFileName(NULL, "Select sensor stream", QString(getenv("HOME")) + QString("/nubot/"),  "Stream Files (*.strm)").toStdString();
                        ok = !cfg.empty() && (!using_sensors || !sstrm.empty());
                    }
                }
            }
        }
    }

//    //frame grab methods
//    QString camoption("Camera"),
//            strmoption("Image stream");
//    QStringList l;
//    l.append(camoption);
//    l.append(strmoption);
//    //get the input choice from the user
//    QString s = QInputDialog::getItem(NULL, "Select Input Method", "Select input method", l, 0, false, &ok);
//    if(ok)
//    {
//        if(s.compare(camoption) == 0)
//            method = DataWrapper::CAMERA;
//        else if(s.compare(strmoption) == 0)
//            method = DataWrapper::STREAM;
//        else
//            method = DataWrapper::STREAM;

//        switch(method) {
//        case DataWrapper::CAMERA:
//            if(QMessageBox::question(NULL, "", "Manually select files?", QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes)
//            {
//                ok = false;
//                lname = QFileDialog::getOpenFileName(NULL, "Select Lookup Table", QString(getenv("HOME")) + QString("/nubot/"),  "LUT Files (*.lut)").toStdString();
//                if(!lname.empty()) {
//                    cfg = QFileDialog::getOpenFileName(NULL, "Select Configuration File", QString(getenv("HOME")) + QString("/nubot/Config/Darwin/"), "config Files (*.cfg)").toStdString();
//                    ok = true;
//                }
//            }
//            else
//            {
//                lname = string(DATA_DIR) +  string("/default.lut");
//                cfg = string(CONFIG_DIR) + string("VisionOptions.cfg");
//            }
//            break;
//        case DataWrapper::STREAM:
//            using_sensors = (QMessageBox::question(NULL, "", "Use sensor log?", QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes);

//            if(QMessageBox::question(NULL, "", "Manually select files?", QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes)
//            {
//                ok = false;
//                istrm = QFileDialog::getOpenFileName(NULL, "Select image stream", QString(getenv("HOME")) + QString("/nubot/"),  "Stream Files (*.strm)").toStdString();
//                if(!istrm.empty()) {
//                    if(using_sensors)
//                        sstrm = QFileDialog::getOpenFileName(NULL, "Select sensor stream", QString(getenv("HOME")) + QString("/nubot/"),  "Stream Files (*.strm)").toStdString();
//                    lname = QFileDialog::getOpenFileName(NULL, "Select Lookup Table", QString(getenv("HOME")) + QString("/nubot/"),  "LUT Files (*.lut)").toStdString();
//                    if(!lname.empty()) {
//                        cfg = QFileDialog::getOpenFileName(NULL, "Select Configuration File", QString(getenv("HOME")) + QString("/nubot/Config/Darwin/"), "config Files (*.cfg)").toStdString();
//                        ok = true;
//                    }
//                }
//            }
//            else
//            {
//                istrm = string(DATA_DIR) + string("/image.strm");
//                if(using_sensors)
//                    sstrm = string(DATA_DIR) + string("/sensor.strm");
//                lname = string(DATA_DIR) + string("/default.lut");
//                cfg = string(CONFIG_DIR) + string("VisionOptions.cfg");
//            }
//            break;
//        }
//    }
}

