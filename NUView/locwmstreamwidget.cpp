#include "locwmstreamwidget.h"
#include <QLabel>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QPushButton>
#include <QPainter>
#include <QImage>
#include <cstring>
#include <QHostAddress>
#include <sstream>
#include "Localisation/Localisation.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"

locwmStreamWidget::locwmStreamWidget(QWidget *parent): QWidget(parent)
{

    locwm = new Localisation();
    objects = new FieldObjects();
    robotName = QString("");
    setWindowTitle(tr("Localisation"));
    setObjectName(tr("Localisation"));
    nameLabel = new QLabel("Robot name: ");
    nameLineEdit = new QLineEdit();
    nameLineEdit->setText("IP ADDRESS");
    connectButton = new QPushButton("Connect");
    disconnectButton = new QPushButton("Disconnect");
    getImageButton = new QPushButton("Get Frame");
    startStreamButton = new QPushButton("Start Stream");
    stopStreamButton = new QPushButton("Stop Stream");
    layout = new QVBoxLayout;
    selectLayout1 = new QHBoxLayout;
    selectLayout1->setAlignment(Qt::AlignTop);
    selectLayout1->addWidget(nameLabel);
    selectLayout1->addWidget(nameLineEdit,2);
    //selectLayout1->addWidget(connectButton,1);
    selectLayout1->addWidget(getImageButton,1);
    selectLayout2 = new QHBoxLayout;
    statusLabel = new QLabel("Status: ");
    statusNetworkLabel = new QLabel("Not connected");
    frameRateLabel = new QLabel("Maximum Frame Rate (FPS): ");
    frameRateMessageLabel = new QLabel("0");
    selectLayout2->setAlignment(Qt::AlignTop);
    selectLayout2->addWidget(statusLabel);
    selectLayout2->addWidget(statusNetworkLabel,2);


    selectLayout3 = new QHBoxLayout;
    selectLayout3->setAlignment(Qt::AlignTop);
    selectLayout3->addWidget(startStreamButton,1);
    selectLayout3->addWidget(stopStreamButton,1);

    selectLayout4 = new QHBoxLayout;
    selectLayout4->setAlignment(Qt::AlignTop);
    selectLayout4->addWidget(frameRateLabel,2);
    selectLayout4->addWidget(frameRateMessageLabel,1);

    //selectLayout2->addWidget(disconnectButton,1);
    layout->addLayout(selectLayout1);
    layout->addLayout(selectLayout2);
    layout->addLayout(selectLayout3);
    layout->addLayout(selectLayout4);
    layout->setAlignment(Qt::AlignLeft);
    //window = new QWidget;
    setLayout(layout);

    //setWidget(window);
    disconnectButton->setEnabled(false);
    connectButton->setEnabled(false);

    tcpSocket = new QTcpSocket(this);
    tcpSocket->setReadBufferSize(0);
    connect(tcpSocket,SIGNAL(readyRead()),this, SLOT(readPendingData()));

    connect(connectButton,SIGNAL(pressed()), this, SLOT(connectToRobot()));
    connect(disconnectButton, SIGNAL(pressed()), this, SLOT(disconnectFromRobot()));
    connect(nameLineEdit, SIGNAL(textChanged(QString)), this, SLOT(updateRobotName(QString)));
    connect(getImageButton,SIGNAL(pressed()),this,SLOT(sendRequestForImage()));
    connect(startStreamButton,SIGNAL(pressed()),&time,SLOT(start()));

    time.setInterval(800);

    connect(stopStreamButton,SIGNAL(pressed()),&time,SLOT(stop()));
    connect(tcpSocket,SIGNAL(connected()),this,SLOT(sendDataToRobot()));


    //time.setSingleShot(true);
    connect(&time,SIGNAL(timeout()),this,SLOT(sendRequestForImage()));


}


//SLOTS:
void locwmStreamWidget::connectToRobot()
{

    //Connect Code HERE
    QHostAddress address = QHostAddress(robotName);
    quint16 port = quint16(16789); //BASE VISON PORT  = 14938
    //udpSocket->bind(QHostAddress::Any, port);
    tcpSocket->connectToHost(address,port,QIODevice::ReadWrite);
    tcpSocket->flush();

    QString text = QString("Connecting to: ");
    text.append(robotName);
    statusNetworkLabel->setText(text);
    //if(udpSocket->writeDatagram (data, 1, address, port) == -1)

/*  const char* data = "1";
      if(tcpSocket->write(data) == -1)
    {
        statusNetworkLabel->setText("Disconnect Error: Unable to send packet.");
        disconnectButton->setEnabled(false);
        connectButton->setEnabled(true);
        datasize = 0;
        if(time.isActive())
        {
            time.stop();
        }
    }
    else
    {
        QString text = QString("Connected to: ");
        text.append(robotName);
        statusNetworkLabel->setText(text);
        disconnectButton->setEnabled(true);
        connectButton->setEnabled(false);
        datasize = 0;
    }
*/
}

void locwmStreamWidget::sendDataToRobot()
{
    const char* data = "1";
    if(tcpSocket->write(data) == -1)
    {
        statusNetworkLabel->setText("Disconnect Error: Unable to send packet.");
        disconnectButton->setEnabled(false);
        connectButton->setEnabled(true);
        datasize = 0;
        if(time.isActive())
        {
            time.stop();
        }
    }
    else
    {
        QString text = QString("Connected to: ");
        text.append(robotName);
        statusNetworkLabel->setText(text);
        disconnectButton->setEnabled(true);
        connectButton->setEnabled(false);
        datasize = 0;
    }
}

void locwmStreamWidget::disconnectFromRobot()
{
    //Disconnect Code HERE
    //QString text = QString("Disconnected");
    //statusNetworkLabel->setText(text);
    disconnectButton->setEnabled(false);
    connectButton->setEnabled(true);
    tcpSocket->disconnectFromHost();
    datasize = 0;
}

void locwmStreamWidget::updateRobotName(const QString name)
{
    connectButton->setEnabled(true);
    robotName = name;
}

void locwmStreamWidget::readPendingData()
{
    if(netdata.isEmpty())
    {
        timeToRecievePacket = QTime();
        timeToRecievePacket.start();
        netdata.append(tcpSocket->readAll());
        //update total bytes to recieve:
        //datasize = (((int)netdata[1])*((int)netdata[2]));
//        QString data;
//        for(int i = 0; i < netdata.size(); i++)
//        {
//            data.append(" ");
//            data.append(QString::number(netdata[i]));
//        }


        std::stringstream buffer;
        buffer.write(reinterpret_cast<char*>(netdata.data()), netdata.size());
        /*
        //QTextStream *stream = new QTextStream(&netdata, QIODevice::ReadOnly);
        //stream->setByteOrder(QDataStream::LittleEndian);

        buffer.read(reinterpret_cast<char*>(&sizeOfSensors), sizeof(sizeOfSensors));
        buffer.read(reinterpret_cast<char*>(&width), sizeof(width));
        buffer.read(reinterpret_cast<char*>(&height), sizeof(height));

        //qDebug() << height << ", " << width;
        imageSize = height*width*4+buffer.tellg()+sizeof(double);
        sensorsSize = sizeOfSensors;
        datasize = imageSize + sensorsSize; // height*width*4+buffer.tellg()+sizeof(double)+ sizeof(NUSensorsData);
        */
        datasize = *(reinterpret_cast<const int*>(netdata.data())) + sizeof(datasize);
        qDebug() << "Data size " << datasize;
        if(datasize < netdata.size())
        {
            qDebug() << "Throwing out data, Size is " << netdata.size() << "reported as" << datasize;
            netdata.clear();
        }
    }
    else
    {
    //QHostAddress sender;
    //quint16 senderPort;

    ////while(tcpSocket->)
    //{
        //READ from Socket
        //netdata.resize(tcpSocket->readBufferSize());
        netdata.append(tcpSocket->readAll());
    //}
    }
    qDebug() << "Current Size" << netdata.size() << "out of" << datasize;
    //emit PacketReady(&datagram);
    if(datasize == netdata.size())
    {
        std::stringstream buffer;
        /*
        buffer.write(reinterpret_cast<char*>(netdata.data()+ sizeof(sizeOfSensors)), imageSize);
        buffer >> image;
        emit rawImageChanged(&image);
        buffer.write(reinterpret_cast<char*>(netdata.data()+ sizeof(sizeOfSensors) + imageSize), sensorsSize);
        buffer >> sensors;
        qDebug() << "Size of Data:" << sensorsSize;
        emit sensorsDataChanged(&sensors);
        */
        buffer.write(reinterpret_cast<char*>(netdata.data() + sizeof(int)), datasize - sizeof(int));
        //buffer >> (*locwm);
        emit locwmDataChanged(locwm);

        if(buffer.str().size() - buffer.tellg() > 0)
        {
            buffer >> (*objects);
            emit fieldObjectDataChanged(objects);
        }

        int mstime = timeToRecievePacket.elapsed();
        time.setInterval(100);
        float frameRate = (float)(1000.00/mstime);
        QString text = QString("Recieved Total Size: ");
        //datasize = datasize +netdata.size();
        text.append(QString::number(netdata.size()));
        text.append(" of ");
        text.append(QString::number(datasize));

        frameRateMessageLabel->setText(QString::number(frameRate));
        statusNetworkLabel->setText(text);
        netdata.clear();
        disconnectFromRobot();
    }

}

void locwmStreamWidget::sendRequestForImage()
{
    if(tcpSocket->state() == QAbstractSocket::UnconnectedState)
    {
        connectToRobot();
    }
}
