 /**    @file connectionwidget.h
 *      @author Aaron Wong
 *
 *      Widget contains functions which allows the program to comunicate and obtain information
 *      from a robot.
 */

#ifndef CONNECTIONWIDGET_H
#define CONNECTIONWIDGET_H

#include <QString>
#include <QDockWidget>
#include <QtNetwork/QUdpSocket>
#include <stdio.h>
#include <iostream>
class QLabel;
class QLineEdit;
class QPushButton;
class QWidget;
class QVBoxLayout;
class QHBoxLayout;

#define Byte unsigned char

/**
  *     A Class for Connection Widget
  */
class ConnectionWidget : public QDockWidget
{
    Q_OBJECT
    public:
            /**
          *     A Constructor for ConnectionWidget class
          *     @param parent The Qwidget pointer parent of the current widget
          */
        ConnectionWidget(QWidget *parent=0);

    public slots:
        /**
          *    A function that initiates comunication with a robot, Must have updated
          *    the robots name before hand.
          */
        void connectToRobot();
        /**
          *    A function that stops comunication with a robot. Must have connected to a
          *     Robot before hand.
          */
        void disconnectFromRobot();
        /**
          *    A function that updates the name of the robot.
          *    Can either be a IP Address e.g 10.0.1.138 or name 'Nubot1.local'
          *    @param name The name of the machine it is to connect to. E.g. '10.0.1.138' or 'NUbot1.local'
          */
        void updateRobotName(const QString name);
        /**
          *    A function that triggered when a packet is recieved.
          *    Once packet is recieved, processing is to be performed on packet.
          *    Or emits a signal to other widgets connected,
          *    to signal whether there is a packet to be processed.
          */
        void readPendingDatagrams();

    signals:
        /**
          *    A function that sends a signal to other connected widgets.
          *    To inform them that there is a new packet to be processed.
          *    @param datagram The QByteArray pointer to a datagram packet to be processed
          */
        void PacketReady(QByteArray* datagram);

    private:
        QString robotName;

        QLabel* nameLabel;
        QLineEdit* nameLineEdit;
        QPushButton* connectButton;
        QPushButton* disconnectButton;
        QVBoxLayout* layout;
        QHBoxLayout* selectLayout1;
        QHBoxLayout* selectLayout2;
        QLabel* frameLabel;
        QLabel* frameNumberLabel;
        QLabel* statusLabel;
        QLabel* statusNetworkLabel;
        QWidget* window;
        QUdpSocket* udpSocket;
};

#endif // CONNECTIONWIDGET_H
