#include "visionstreamwidget.h"
#include <QLabel>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QPushButton>
#include <QPainter>
#include <QImage>
#include <cstring>
#include <QHostAddress>
#include <sstream>


visionStreamWidget::visionStreamWidget(QMdiArea* parentMdiWidget, QWidget *parent): QWidget(parent)
{
    robotName = QString("");
    setWindowTitle(tr("Vision"));
    setObjectName(tr("Vision"));
    nameLabel = new QLabel("Robot name: ", this);
    nameLineEdit = new QLineEdit(this);
    nameLineEdit->setText("IP ADDRESS");
    connectButton = new QPushButton("Connect",this);
    disconnectButton = new QPushButton("Disconnect",this);
    getImageButton = new QPushButton("Get an &Image",this);
    startStreamButton = new QPushButton("Start Stream",this);
    stopStreamButton = new QPushButton("Stop Stream",this);
    layout = new QVBoxLayout(this);
    selectLayout1 = new QHBoxLayout(this);
    selectLayout1->setAlignment(Qt::AlignTop);
    selectLayout1->addWidget(nameLabel);
    selectLayout1->addWidget(nameLineEdit,2);
    //selectLayout1->addWidget(connectButton,1);
    selectLayout1->addWidget(getImageButton,1);
    selectLayout2 = new QHBoxLayout(this);
    statusLabel = new QLabel("Status: ",this);
    statusNetworkLabel = new QLabel("Not connected",this);
    frameRateLabel = new QLabel("Maximum Frame Rate (FPS): ",this);
    frameRateMessageLabel = new QLabel("0",this);
    selectLayout2->setAlignment(Qt::AlignTop);
    selectLayout2->addWidget(statusLabel);
    selectLayout2->addWidget(statusNetworkLabel,2);


    selectLayout3 = new QHBoxLayout(this);
    selectLayout3->setAlignment(Qt::AlignTop);
    selectLayout3->addWidget(startStreamButton,1);
    selectLayout3->addWidget(stopStreamButton,1);

    selectLayout4 = new QHBoxLayout(this);
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

    image = new NUImage();
    sensors = new NUSensorsData();
}

visionStreamWidget::~visionStreamWidget()
{
    delete image;
    delete sensors;
}

//SLOTS:
void visionStreamWidget::connectToRobot()
{

    //Connect Code HERE
    QHostAddress address = QHostAddress(robotName);
    quint16 port = quint16(14938); //BASE VISON PORT  = 14938
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

void visionStreamWidget::sendDataToRobot()
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

void visionStreamWidget::disconnectFromRobot()
{
    //Disconnect Code HERE
    //QString text = QString("Disconnected");
    //statusNetworkLabel->setText(text);
    disconnectButton->setEnabled(false);
    connectButton->setEnabled(true);
    tcpSocket->disconnectFromHost();
    datasize = 0;
}

void visionStreamWidget::updateRobotName(const QString name)
{
    connectButton->setEnabled(true);
    robotName = name;
}

void visionStreamWidget::readPendingData()
{
    int height,width, sizeOfSensors;
    if(netdata.isEmpty())
    {
        timeToRecievePacket = QTime();
        timeToRecievePacket.start();
        netdata.append(tcpSocket->readAll());
        //update total bytes to recieve:
        //datasize = (((int)netdata[1])*((int)netdata[2]));
        QString data;
        for(int i = 0; i < netdata.size(); i++)
        {
            data.append(" ");
            data.append(QString::number(netdata[i]));
        }

        std::stringstream buffer;
        buffer.write(reinterpret_cast<char*>(netdata.data()), netdata.size());

        //QTextStream *stream = new QTextStream(&netdata, QIODevice::ReadOnly);
        //stream->setByteOrder(QDataStream::LittleEndian);

        buffer.read(reinterpret_cast<char*>(&sizeOfSensors), sizeof(sizeOfSensors));
        std::streampos img_begin = buffer.tellg();
        NUImage::Header image_header;
        NUImage::Version img_version = NUImage::VERSION_UNKNOWN;
        buffer.read(reinterpret_cast<char*>(&image_header), sizeof(image_header));
        if(NUImage::validHeader(image_header))
        {
            img_version = image_header.version;
        }
        else
        {
            buffer.seekg(img_begin);
        }
//        qDebug() << "Image Version: " << img_version;
        buffer.read(reinterpret_cast<char*>(&width), sizeof(width));
        buffer.read(reinterpret_cast<char*>(&height), sizeof(height));

//        qDebug() << height << ", " << width;
        imageSize = height*width*4+buffer.tellg()+sizeof(double)+sizeof(bool);
        sensorsSize = sizeOfSensors;
        datasize = imageSize + sensorsSize; // height*width*4+buffer.tellg()+sizeof(double)+ sizeof(NUSensorsData);
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

        //emit PacketReady(&datagram);
        if(datasize == netdata.size())
        {
            std::stringstream buffer;
            buffer.write(reinterpret_cast<char*>(netdata.data()+ sizeof(sizeOfSensors)), imageSize);
            buffer >> (*image);
            emit rawImageChanged(image);
            buffer.write(reinterpret_cast<char*>(netdata.data()+ sizeof(sizeOfSensors) + imageSize), sensorsSize);
            buffer >> (*sensors);
            qDebug() << "Size of Data:" << sensorsSize;
            emit sensorsDataChanged(sensors);

            int mstime = timeToRecievePacket.elapsed();
            time.setInterval(0);
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

}

void visionStreamWidget::sendRequestForImage()
{
    if(tcpSocket->state() == QAbstractSocket::UnconnectedState)
    {
        connectToRobot();
    }
}
