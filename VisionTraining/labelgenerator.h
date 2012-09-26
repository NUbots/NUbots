#ifndef LABELGENERATOR_H
#define LABELGENERATOR_H

#include <QDialog>
#include <string>

using namespace std;

namespace Ui {
    class LabelGenerator;
}

class LabelGenerator : public QDialog
{
    Q_OBJECT

public:
    explicit LabelGenerator(QWidget *parent = 0);
    ~LabelGenerator();
    
    bool run(const string& dir);
    
private slots:
    void setCancelled() {cancelled = true;}

private:
    Ui::LabelGenerator *ui;
    bool cancelled;
};

#endif // LABELGENERATOR_H
