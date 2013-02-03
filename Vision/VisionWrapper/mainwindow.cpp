#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <boost/foreach.hpp>
#include <QImage>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    image = new QImage(QSize(320, 240), QImage::Format_RGB888);
    base_image = new QImage(QSize(320, 240), QImage::Format_RGB888);
    //create layers
    int y=0;
    for(int id=0; id<DBID_INVALID; id++) {
        QCheckBox* newbox = new QCheckBox(getDebugIDName(getDebugIDFromInt(id)).c_str());
        QRect geom = ui->layersScrollArea->geometry();
        geom.setHeight(newbox->height());
        if(layer_selections.empty()) {
            y = geom.y();
        }
        else {
            y += geom.height();
        }
        geom.setY(y);
        newbox->setGeometry(geom);
        layer_selections[getDebugIDFromInt(id)] = newbox;
    }
}

MainWindow::~MainWindow()
{
    delete ui;
    delete image;
    delete base_image;
    for(int id=0; id<DBID_INVALID; id++) {
        delete layer_selections[getDebugIDFromInt(id)];
    }
}

void MainWindow::clearLayers()
{
    for(int i=0; i<DBID_INVALID; i++) {
        DEBUG_ID id = getDebugIDFromInt(i);
        points[id].clear();
        lines[id].clear();
        rectangles[id].clear();
        circles[id].clear();
        polygons[id].clear();
    }
    refresh();
}

void MainWindow::refresh()
{
    *image = *base_image;
    QPainter painter(image);
    for(int i=0; i<DBID_INVALID; i++) {
        DEBUG_ID id = getDebugIDFromInt(i);
        if(layer_selections[id]->isChecked()) {
            for(vector<pair<QPointF, QColor> >::iterator it = points[id].begin(); it<points[id].end(); it++) {
                painter.setPen(it->second);
                painter.drawPoint(it->first);
            }
            for(vector<pair<QLineF, QColor> >::iterator it = lines[id].begin(); it<lines[id].end(); it++) {
                painter.setPen(it->second);
                painter.drawLine(it->first);
            }
            for(vector<pair<QRectF, QColor> >::iterator it = rectangles[id].begin(); it<rectangles[id].end(); it++) {
                painter.setPen(it->second);
                painter.drawRect(it->first);
            }
            for(vector<pair<Circle, QColor> >::iterator it = circles[id].begin(); it<circles[id].end(); it++) {
                painter.setPen(it->second);
                painter.drawEllipse(it->first.m_centre, it->first.m_radius, it->first.m_radius);
            }
            for(vector<pair<Polygon, QColor> >::iterator it = polygons[id].begin(); it<polygons[id].end(); it++) {
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

    for(map<QString, QwtPlot*>::iterator pit = plots.begin(); pit != plots.end(); pit++)
        pit->second->show();
}

void MainWindow::addToLayer(DEBUG_ID id, const QPointF& item, QColor colour)
{
    points[id].push_back(pair<QPointF, QColor>(item, colour));
}

void MainWindow::addToLayer(DEBUG_ID id, const QLineF& item, QColor colour)
{
    lines[id].push_back(pair<QLineF, QColor>(item, colour));
}

void MainWindow::addToLayer(DEBUG_ID id, const QRectF& item, QColor colour)
{
    rectangles[id].push_back(pair<QRectF, QColor>(item, colour));
}

void MainWindow::addToLayer(DEBUG_ID id, const Circle& item, QColor colour)
{
    circles[id].push_back(pair<Circle, QColor>(item, colour));
}

void MainWindow::addToLayer(DEBUG_ID id, const Polygon& item, QColor colour)
{
    polygons[id].push_back(pair<Polygon, QColor>(item, colour));
}

void MainWindow::addToLayer(DEBUG_ID id, const vector<QPointF>& items, QColor colour)
{
    BOOST_FOREACH(const QPointF& item, items) {
        addToLayer(id, item, colour);
    }
}

void MainWindow::addToLayer(DEBUG_ID id, const vector<QLineF>& items, QColor colour)
{
    BOOST_FOREACH(const QLineF& item, items) {
        addToLayer(id, item, colour);
    }
}

void MainWindow::addToLayer(DEBUG_ID id, const vector<QRectF>& items, QColor colour)
{
    BOOST_FOREACH(const QRectF& item, items) {
        addToLayer(id, item, colour);
    }
}

void MainWindow::addToLayer(DEBUG_ID id, const vector<Circle>& items, QColor colour)
{
    BOOST_FOREACH(const Circle& item, items) {
        addToLayer(id, item, colour);
    }
}

void MainWindow::addToLayer(DEBUG_ID id, const vector<Polygon>& items, QColor colour)
{
    BOOST_FOREACH(const Polygon& item, items) {
        addToLayer(id, item, colour);
    }
}

void MainWindow::setPlot(QString name, QwtPlotCurve curve)
{
    QwtPlot* plot = new QwtPlot(QwtText(name), ui->windowFrame);

    plot->setAxisAutoScale(QwtPlot::xBottom, true);
    plot->setAxisAutoScale(QwtPlot::yLeft, true);
    plot->setGeometry(0, 0, 320, 240);

    curve.attach(plot);

    plot->replot();

    plots[name] = plot;
}
