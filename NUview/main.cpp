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
    MainWindow w;
    w.show();
    return a.exec();
}
