#include "connectionwidget.h"
#include <QLabel>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QPushButton>
#include <QPainter>
#include <QImage>
#include <cstring>

ConnectionWidget::ConnectionWidget(QWidget *parent): QDockWidget(parent)
{
    robotName = QString("");
    setWindowTitle(tr("Conection Manager"));
    setObjectName(tr("ConectionManager"));
    nameLabel = new QLabel("Robot name: ");
    nameLineEdit = new QLineEdit();
    nameLineEdit->setText("Type Name: *.local");
    connectButton = new QPushButton("Connect");
    disconnectButton = new QPushButton("Disconnect");
    layout = new QVBoxLayout;
    selectLayout1 = new QHBoxLayout;
    selectLayout1->setAlignment(Qt::AlignTop);
    selectLayout1->addWidget(nameLabel);
    selectLayout1->addWidget(nameLineEdit,2);
    selectLayout1->addWidget(connectButton,1);

    selectLayout2 = new QHBoxLayout;;
    statusLabel = new QLabel("Status: ");
    statusNetworkLabel = new QLabel("Not connected");
    selectLayout2->setAlignment(Qt::AlignTop);
    selectLayout2->addWidget(statusLabel);
    selectLayout2->addWidget(statusNetworkLabel,2);
    selectLayout2->addWidget(disconnectButton,1);
    layout->addLayout(selectLayout1);
    layout->addLayout(selectLayout2);
    layout->setAlignment(Qt::AlignLeft);
    window = new QWidget;
    window->setLayout(layout);

    setWidget(window);
    disconnectButton->setEnabled(false);
    connectButton->setEnabled(false);

    udpSocket = new QUdpSocket(this);
    udpSocket->setReadBufferSize(12000);
    connect(udpSocket, SIGNAL(readyRead()), this, SLOT(readPendingDatagrams()));

    connect(connectButton,SIGNAL(pressed()), this, SLOT(connectToRobot()));
    connect(disconnectButton, SIGNAL(pressed()), this, SLOT(disconnectFromRobot()));
    connect(nameLineEdit, SIGNAL(textChanged(QString)), this, SLOT(updateRobotName(QString)));
}


//SLOTS:
void ConnectionWidget::connectToRobot()
{

    //Connect Code HERE
    QHostAddress address = QHostAddress(robotName);
    quint16 port = quint16(8533);
    udpSocket->bind(QHostAddress::Any, port);
    const char* data = "1";

    if(udpSocket->writeDatagram (data, 1, address, port) == -1)
    {
        statusNetworkLabel->setText("Disconnect Error: Unable to send packet.");
        disconnectButton->setEnabled(false);
        connectButton->setEnabled(true);
    }
    else
    {
        QString text = QString("Connected to: ");
        text.append(robotName);
        statusNetworkLabel->setText(text);
        disconnectButton->setEnabled(true);
        connectButton->setEnabled(false);
    }

}

void ConnectionWidget::disconnectFromRobot()
{

    //Disconnect Code HERE
    quint16 port = quint16(8533);
    QHostAddress address = QHostAddress(robotName);
    const char* data = "0";
    if(udpSocket->writeDatagram (data, 1, address, port) == -1)
    {
        statusNetworkLabel->setText("Disconnect Error: Unable to send packet.");
    }
    else
    {
        QString text = QString("Disconnected");
        statusNetworkLabel->setText(text);
        disconnectButton->setEnabled(false);
        connectButton->setEnabled(true);
        udpSocket->disconnectFromHost();
    }


}

void ConnectionWidget::updateRobotName(const QString name)
{
    connectButton->setEnabled(true);
    robotName = name;
}

void ConnectionWidget::readPendingDatagrams()
{

    QByteArray datagram;
    QHostAddress sender;
    quint16 senderPort;

    while(udpSocket->hasPendingDatagrams())
    {
        //READ from Socket
        datagram.resize(udpSocket->pendingDatagramSize());
        udpSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);
    }

    QString text = QString("Recieved Size: ");

    text.append(QString::number(datagram.size()));
    statusNetworkLabel->setText(text);
    emit PacketReady(&datagram);
}
