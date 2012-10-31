#include "mainwindow.h"
#include <QApplication>

int main()
{
    //The MainWindow class controls the UI logic
    QApplication app(NULL);
    MainWindow mw;
    mw.show();
    return app.exec();
}
