/**
 * \mainpage
 * @author NUbots
 *
 * NuViewer used for Debuggin purposes on the Aldebaran Nao Robots
 */

#include <QtGui/QApplication>
#include "mainwindow.h"
#define NUVIEW


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    
    // load default style sheet
    QFile file("://styles/default.qss");
    file.open(QFile::ReadOnly);
    QString style = QString(file.readAll());
    a.setStyleSheet(style);
    file.close();
    
    MainWindow w;
    w.show();
    return a.exec();
}
