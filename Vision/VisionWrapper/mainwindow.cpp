#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <string>
#include <sstream>
#include <boost/foreach.hpp>
#include <QPainter>
#include <QScrollBar>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    m_finished = m_next = false;
    ui->setupUi(this);
    canvas = new QImage(QSize(320, 240), QImage::Format_RGB888);
    canvas->fill(QColor(0,0,0));

    layers_layout = new QVBoxLayout();
    layers_box = new QWidget(ui->layersScrollArea);
    layers_box->setLayout(layers_layout);
    ui->layersScrollArea->setWidget(layers_box);
    //create layers
    for(int i=0; i<DBID_INVALID; i++) {
        DEBUG_ID id = getDebugIDFromInt(i);
        QCheckBox* newbox = new QCheckBox(getDebugIDName(getDebugIDFromInt(id)).c_str(), ui->layersScrollArea);
        QObject::connect(newbox, SIGNAL(clicked()), this, SLOT(refresh()));
        layers_layout->addWidget(newbox);
        layer_selections[id] = newbox;
    }

    windows_box = new QWidget(ui->windowsScrollArea);
    ui->windowsScrollArea->setWidget(windows_box);
    windows_layout = new QGridLayout(windows_box);
    windows_box->setLayout(windows_layout);

    view = new QGraphicsView(ui->windowsScrollArea);

    view->setFixedSize(320,240);
    view->setContentsMargins(0,0,0,0);
    view->setFrameShape(QFrame::NoFrame);
    windows_layout->addWidget(view);
    scene = new QGraphicsScene;
    scene->addItem(new QGraphicsPixmapItem(QPixmap::fromImage(*canvas)));
    view->setScene(scene);


    QObject::connect(ui->next_pb, SIGNAL(clicked()), this, SLOT(setNext()));
    QObject::connect(ui->exit_pb, SIGNAL(clicked()), this, SLOT(setFinished()));
}

MainWindow::~MainWindow()
{
    delete scene;
    delete view;
    delete canvas;
    delete ui;
    for(int id=0; id<DBID_INVALID; id++) {
        delete layer_selections[getDebugIDFromInt(id)];
    }
}

void MainWindow::setFrameNo(int n)
{
    std::stringstream s;
    s << "Frame: " << n;
    ui->label_frame->setText(s.str().c_str());
}

void MainWindow::clearLayers()
{
    for(int i=0; i<DBID_INVALID; i++) {
        DEBUG_ID id = getDebugIDFromInt(i);

        images[id].clear();
        points[id].clear();
        lines[id].clear();
        rectangles[id].clear();
        circles[id].clear();
        polygons[id].clear();

    }

    //clear out plots
    for(map<QString, pair<QwtPlot*, QwtPlotCurve*> >::iterator it = plots.begin(); it != plots.end(); it++) {
        delete it->second.second;
        delete it->second.first;
    }
    plots.clear();

    refresh();
}

void MainWindow::refresh()
{
    qDeleteAll( scene->items() );
    //reset image
    canvas->fill(QColor(Qt::black));
    //draw over image
    QPainter painter(canvas);
    for(int i=0; i<DBID_INVALID; i++) {
        DEBUG_ID id = getDebugIDFromInt(i);
        if(layer_selections[id]->isChecked()) {
            for(vector<pair<QImage, float> >::iterator it = images[id].begin(); it<images[id].end(); it++) {
                painter.setOpacity(it->second);
                painter.drawImage(QPoint(0,0), it->first);
            }
            for(vector<pair<QPointF, QPen> >::iterator it = points[id].begin(); it<points[id].end(); it++) {
                painter.setPen(it->second);
                painter.drawPoint(it->first);
            }
            for(vector<pair<QLineF, QPen> >::iterator it = lines[id].begin(); it<lines[id].end(); it++) {
                painter.setPen(it->second);
                painter.drawLine(it->first);
            }
            for(vector<pair<QRectF, QPen> >::iterator it = rectangles[id].begin(); it<rectangles[id].end(); it++) {
                painter.setPen(it->second);
                painter.drawRect(it->first);
            }
            for(vector<pair<QCircle, QPen> >::iterator it = circles[id].begin(); it<circles[id].end(); it++) {
                painter.setPen(it->second);
                painter.drawEllipse(it->first.m_centre, it->first.m_radius, it->first.m_radius);
            }
            for(vector<pair<Polygon, QPen> >::iterator it = polygons[id].begin(); it<polygons[id].end(); it++) {
                painter.setPen(it->second);
                if(it->first.m_filled) {
                    painter.drawPolygon(it->first.m_polygon);
                }
                else {
                    painter.drawPolyline(it->first.m_polygon);
                }
            }
        }
    }

    //render image
    scene->addItem(new QGraphicsPixmapItem(QPixmap::fromImage(*canvas)));
    view->setScene(scene);

    view->show();

    //plots
    for(map<QString, pair<QwtPlot*, QwtPlotCurve*> >::iterator pit = plots.begin(); pit != plots.end(); pit++)
        pit->second.first->show();
}

void MainWindow::addToLayer(DEBUG_ID id, const QImage& img, float alpha)
{
    images[id].push_back(pair<QImage, float>(img, alpha));
}

void MainWindow::addToLayer(DEBUG_ID id, const QPointF& item, QPen pen)
{
    points[id].push_back(pair<QPointF, QPen>(item, pen));
}

void MainWindow::addToLayer(DEBUG_ID id, const QLineF& item, QPen pen)
{
    lines[id].push_back(pair<QLineF, QPen>(item, pen));
}

void MainWindow::addToLayer(DEBUG_ID id, const QRectF& item, QPen pen)
{
    rectangles[id].push_back(pair<QRectF, QPen>(item, pen));
}

void MainWindow::addToLayer(DEBUG_ID id, const QCircle& item, QPen pen)
{
    circles[id].push_back(pair<QCircle, QPen>(item, pen));
}

void MainWindow::addToLayer(DEBUG_ID id, const Polygon& item, QPen pen)
{
    polygons[id].push_back(pair<Polygon, QPen>(item, pen));
}

void MainWindow::addToLayer(DEBUG_ID id, const vector<QPointF>& items, QPen pen)
{
    BOOST_FOREACH(const QPointF& item, items) {
        addToLayer(id, item, pen);
    }
}

void MainWindow::addToLayer(DEBUG_ID id, const vector<QLineF>& items, QPen pen)
{
    BOOST_FOREACH(const QLineF& item, items) {
        addToLayer(id, item, pen);
    }
}

void MainWindow::addToLayer(DEBUG_ID id, const vector<QRectF>& items, QPen pen)
{
    BOOST_FOREACH(const QRectF& item, items) {
        addToLayer(id, item, pen);
    }
}

void MainWindow::addToLayer(DEBUG_ID id, const vector<QCircle>& items, QPen pen)
{
    BOOST_FOREACH(const QCircle& item, items) {
        addToLayer(id, item, pen);
    }
}

void MainWindow::addToLayer(DEBUG_ID id, const vector<Polygon>& items, QPen pen)
{
    BOOST_FOREACH(const Polygon& item, items) {
        addToLayer(id, item, pen);
    }
}

void MainWindow::setPlot(QString name, QwtPlotCurve* curve)
{
    //delete old curve and plot
    map<QString, pair<QwtPlot*, QwtPlotCurve*> >::iterator it = plots.find(name);

    if(it != plots.end()) {
        delete it->second.second;
        delete it->second.first;
    }

    QwtPlot* plot = new QwtPlot(QwtText(name), ui->windowsScrollArea);

    plot->setAxisAutoScale(QwtPlot::xBottom, true);
    plot->setAxisAutoScale(QwtPlot::yLeft, true);
    plot->setFixedSize(320,240);

    curve->attach(plot);

    //windows_layout->addWidget(plot, row_num, column_num);
    windows_layout->addWidget(plot);

    plot->replot();

    plots[name] = pair<QwtPlot*, QwtPlotCurve*>(plot, curve);
}
