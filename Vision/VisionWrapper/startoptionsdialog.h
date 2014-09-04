#ifndef STARTOPTIONSDIALOG_H
#define STARTOPTIONSDIALOG_H

#include <QDialog>

#include <iostream>

namespace Ui {
class StartOptionsDialog;
}

class StartOptionsDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit StartOptionsDialog(QWidget *parent = 0);
    ~StartOptionsDialog();

    bool isValid() {return valid;}
    bool isInputCamera() {return camera;}
    bool isAutomatic() {return automatic;}
    bool isFolderBased() {return folder;}
    bool isUsingSensors() {return sensorstream;}
    
private slots:
    void setValid() {valid=true;}
    void setInValid() {valid=false;}
    void setInputCamera(bool v) {camera=v;}
    void setAutomatic(bool v) {automatic=v;}
    void setFolderBased(bool v) {folder=v;}
    void setSensorStream(bool v) {sensorstream=v;}

private:
    bool valid;
    bool camera;
    bool sensorstream;
    bool automatic;
    bool folder;

    Ui::StartOptionsDialog *ui;
};

#endif // STARTOPTIONSDIALOG_H
