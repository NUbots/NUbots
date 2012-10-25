#ifndef OPTIMISERSELECTWINDOW_H
#define OPTIMISERSELECTWINDOW_H

#include <QDialog>
#include <vector>

namespace Ui {
class OptimiserSelectWindow;
}

class OptimiserSelectWindow : public QDialog
{
    Q_OBJECT
    
public:
    explicit OptimiserSelectWindow(QWidget *parent = 0);
    ~OptimiserSelectWindow();

    std::vector<bool> getOptions();

private slots:
    void setPosts(bool b) {m_posts_on = b;}
    void setBalls(bool b) {m_balls_on = b;}
    void setObst(bool b) {m_obst_on = b;}
    void setLines(bool b) {m_lines_on = b;}
    void setGen(bool b) {m_gen_on = b;}
    void setAccepted() {m_acc = true;}
    void setRejected() {m_rej = true;}
    
private:
    Ui::OptimiserSelectWindow *ui;
    bool m_posts_on,
        m_balls_on,
        m_obst_on,
        m_lines_on,
        m_gen_on,
        m_acc,
        m_rej;

};

#endif // OPTIMISERSELECTWINDOW_H
