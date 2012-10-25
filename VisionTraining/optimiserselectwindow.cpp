#include "optimiserselectwindow.h"
#include "ui_optimiserselectwindow.h"

OptimiserSelectWindow::OptimiserSelectWindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::OptimiserSelectWindow)
{
    ui->setupUi(this);
    m_posts_on = m_balls_on = m_obst_on = m_lines_on = m_gen_on = m_acc = m_rej = false;
    QObject::connect(ui->postCB, SIGNAL(clicked(bool)), this, SLOT(setPosts(bool)));
    QObject::connect(ui->ballCB, SIGNAL(clicked(bool)), this, SLOT(setBalls(bool)));
    QObject::connect(ui->obstCB, SIGNAL(clicked(bool)), this, SLOT(setObst(bool)));
    QObject::connect(ui->lineCB, SIGNAL(clicked(bool)), this, SLOT(setLines(bool)));
    QObject::connect(ui->genCB, SIGNAL(clicked(bool)), this, SLOT(setGen(bool)));
    QObject::connect(ui->buttonBox, SIGNAL(accepted()), this, SLOT(setAccepted()));
    QObject::connect(ui->buttonBox, SIGNAL(rejected()), this, SLOT(setRejected()));
}

OptimiserSelectWindow::~OptimiserSelectWindow()
{
    delete ui;
}

std::vector<bool> OptimiserSelectWindow::getOptions()
{
    std::vector<bool> vb;
    while(!m_acc && !m_rej) {
        QApplication::processEvents();
    }
    if(m_acc) {
        vb.push_back(m_posts_on);
        vb.push_back(m_balls_on);
        vb.push_back(m_obst_on);
        vb.push_back(m_lines_on);
        vb.push_back(m_gen_on);
        return vb;
    }
    else {
        return vb;
    }
}
