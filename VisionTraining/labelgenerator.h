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
    void cancel() {cancelled = true;}

private:
    Ui::LabelGenerator *ui; //! @var The GUI pointer.
    bool cancelled;         //! @var Whether the user hit cancel.
};

#endif // LABELGENERATOR_H
