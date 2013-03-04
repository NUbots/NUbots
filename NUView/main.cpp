/**
 * \mainpage
 * @author NUbots
 *
 * NuViewer used for Debuggin purposes on the Aldebaran Nao Robots
 */

#include <QtGui/QApplication>
#include "mainwindow.h"
#define NUVIEW

//for catching exceptions
class MyApplication : public QApplication {
public:
    MyApplication(int& argc, char ** argv) : QApplication(argc, argv) { }
    virtual ~MyApplication() { }

    // reimplemented from QApplication so we can throw exceptions in slots
    virtual bool notify(QObject * receiver, QEvent * event) {
        try {
        return QApplication::notify(receiver, event);
        }
        catch(const std::exception& e) {
            qCritical() << "Exception thrown:" << e.what();
        }
        catch (const std::string& ex) {
            qCritical() << "Manual exception thrown:" << ex.c_str();
        }
        catch (const char* str) {
            qCritical() << "Manual exception thrown:" << str;
        }
        catch (...) {
            qCritical() << "Unknown exception thrown.";
        }
        return false;
    }
};


int main(int argc, char *argv[])
{
    MyApplication a(argc, argv);
    
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
