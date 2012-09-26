#ifndef LABELEDITOR_H
#define LABELEDITOR_H

#include <QMainWindow>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QSlider>
#include <vector>

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

private slots:
    void halt() {halted=true;}
    
private:
    Ui::LabelEditor *ui;
    vector<QLabel*> labels;
    bool halted;
};

#endif // LABELEDITOR_H
