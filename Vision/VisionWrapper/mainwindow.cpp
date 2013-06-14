#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <string>
#include <sstream>
#include <boost/foreach.hpp>
#include <QPainter>
#include <QScrollBar>
#include <qwt/qwt_legend.h>
#include <sstream>
#include "linesegmentscurve.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    m_finished = m_next = m_continuous = false;
    ui->setupUi(this);

    current_window = 0;

    layers_layout = new QVBoxLayout();
    layers_box = new QWidget(ui->layersScrollArea);
    layers_box->setLayout(layers_layout);
    ui->layersScrollArea->setWidget(layers_box);
    //create layers
    for(int i=0; i<numDebugIDs(); i++) {
        DEBUG_ID id = debugIDFromInt(i);
        QCheckBox* newbox = new QCheckBox(debugIDName(id).c_str(), ui->layersScrollArea);
        QObject::connect(newbox, SIGNAL(clicked()), this, SLOT(refresh()));
        layers_layout->addWidget(newbox);
        layer_boxes[id] = newbox;
    }

    windows_box = new QWidget(ui->windowsScrollArea);
    ui->windowsScrollArea->setWidget(windows_box);
    windows_layout = new QGridLayout(windows_box);
    windows_box->setLayout(windows_layout);

    //two canvases
    for(size_t i=0; i<NUM_CANVASES; i++) {
        std::stringstream ss;
        ss << "Window " << i;
        ui->windowComboBox->addItem(ss.str().c_str());
        canvases.push_back(new QImage(QSize(320, 240), QImage::Format_RGB888));
        canvases.back()->fill(QColor(0,0,0));
        views.push_back(new QGraphicsView(ui->windowsScrollArea));
        //views.back()->setFixedSize(320,240);
        views.back()->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        views.back()->setMinimumSize(320,340);
        views.back()->setContentsMargins(0,0,0,0);
        views.back()->setFrameShape(QFrame::NoFrame);
        windows_layout->addWidget(views.back(), 0, i);
        scenes.push_back(new QGraphicsScene);
        scenes.back()->addItem(new QGraphicsPixmapItem(QPixmap::fromImage(*(canvases.back()))));
        views.back()->setScene(scenes.back());
        layer_selections.push_back(std::map<DEBUG_ID, bool>());
    }

    for(size_t i = p1; i<NUM_PLOTS; i++) {
        PLOTWINDOW win = winFromInt(i);
        plots[win] = new QwtPlot(ui->windowsScrollArea);
        plots[win]->setAxisAutoScale(QwtPlot::xBottom, true);
        plots[win]->setAxisAutoScale(QwtPlot::yLeft, true);
        plots[win]->setFixedSize(320,240);
        plots[win]->insertLegend(new QwtLegend(), QwtPlot::BottomLegend);
        windows_layout->addWidget(plots[win], (i + NUM_CANVASES)/2, (i+NUM_CANVASES)%2);
        zoomers[win] = new QwtPlotZoomer(plots[win]->canvas());
    }

    connect(ui->next_pb, SIGNAL(clicked()), this, SLOT(setNext()));
    connect(ui->prev_pb, SIGNAL(clicked()), this, SLOT(setPrev()));
    connect(ui->contButton, SIGNAL(clicked()), this, SLOT(toggleContinuous()));
    connect(ui->exit_pb, SIGNAL(clicked()), this, SLOT(setFinished()));
    connect(ui->windowComboBox, SIGNAL(activated(int)), this, SLOT(setWindow(int)));
}

MainWindow::~MainWindow()
{
    BOOST_FOREACH(QGraphicsScene* s, scenes) {
        delete s;
    }
    BOOST_FOREACH(QGraphicsView* v, views) {
        delete v;
    }
    BOOST_FOREACH(QImage* c, canvases) {
        delete c;
    }

    delete ui;
    for(int id=0; id<numDebugIDs(); id++) {
        delete layer_boxes[debugIDFromInt(id)];
    }

//    for(std::map<PLOTWINDOW, QwtPlotMagnifier*>::iterator it = magnifiers.begin(); it != magnifiers.end(); it++) {
//        delete it->second;
//    }
    for(std::map<PLOTWINDOW, QwtPlotZoomer*>::iterator it = zoomers.begin(); it != zoomers.end(); it++) {
        delete it->second;
    }
    plots.clear();
    for(std::map<PLOTWINDOW, QwtPlot*>::iterator it = plots.begin(); it != plots.end(); it++) {
        delete it->second;
    }
    plots.clear();
    for(std::map<QString, QwtPlotCurve*>::iterator it = curves.begin(); it != curves.end(); it++) {
        delete it->second;
    }
    curves.clear();
}

void MainWindow::setFrameNo(int n)
{
    std::stringstream s;
    s << "Frame: " << n;
    ui->label_frame->setText(s.str().c_str());
}

void MainWindow::resetFlags()
{
     m_next = m_continuous;
     m_prev = false;
     m_finished = false;
}

void MainWindow::clearLayers()
{
    for(int i=0; i<numDebugIDs(); i++) {
        DEBUG_ID id = debugIDFromInt(i);
        images[id].clear();
        points[id].clear();
        lines[id].clear();
        rectangles[id].clear();
        circles[id].clear();
        polygons[id].clear();
    }

    //clear out curves
    for(std::map<QString, QwtPlotCurve*>::iterator it = curves.begin(); it != curves.end(); it++) {
        delete it->second;
    }
    curves.clear();

    refresh();
}

void MainWindow::refresh()
{
    for(int i=0; i<numDebugIDs(); i++) {
        DEBUG_ID id = debugIDFromInt(i);
        layer_selections[current_window][id] = layer_boxes[id]->isChecked();
    }

    //get size of largest image
    QSize size(0,0);
    for(int i=0; i<numDebugIDs(); i++) {
        DEBUG_ID id = debugIDFromInt(i);
        for(vector<pair<QImage, float> >::iterator it = images[id].begin(); it<images[id].end(); it++) {
            size = QSize(std::max(size.width(), it->first.width()), std::max(size.height(), it->first.height()));
        }
    }

    for(size_t c=0; c<NUM_CANVASES; c++) {
        qDeleteAll( scenes[c]->items() );

        //reset image
        delete canvases[c];
        canvases[c] = new QImage(size, QImage::Format_RGB888);
        canvases[c]->fill(QColor(Qt::black));

        //draw over image
        QPainter painter(canvases[c]);
        for(int i=0; i<numDebugIDs(); i++) {
            DEBUG_ID id = debugIDFromInt(i);
            if(layer_selections[c][id]) {
                for(std::vector<std::pair<QImage, float> >::iterator it = images[id].begin(); it<images[id].end(); it++) {
                    painter.setOpacity(it->second);
                    painter.drawImage(QPoint(0,0), it->first);
                }
                for(std::vector<std::pair<QPointF, QPen> >::iterator it = points[id].begin(); it<points[id].end(); it++) {
                    painter.setPen(it->second);
                    painter.drawPoint(it->first);
                }
                for(std::vector<std::pair<QLineF, QPen> >::iterator it = lines[id].begin(); it<lines[id].end(); it++) {
                    painter.setPen(it->second);
                    painter.drawLine(it->first);
                }
                for(std::vector<std::pair<QRectF, QPen> >::iterator it = rectangles[id].begin(); it<rectangles[id].end(); it++) {
                    painter.setPen(it->second);
                    painter.drawRect(it->first);
                }
                for(std::vector<std::pair<QCircle, QPen> >::iterator it = circles[id].begin(); it<circles[id].end(); it++) {
                    painter.setPen(it->second);
                    painter.drawEllipse(it->first.m_centre, it->first.m_radius, it->first.m_radius);
                }
                for(std::vector<std::pair<Polygon, QPen> >::iterator it = polygons[id].begin(); it<polygons[id].end(); it++) {
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
        scenes[c]->addItem(new QGraphicsPixmapItem(QPixmap::fromImage(*(canvases[c]))));
        views[c]->setScene(scenes[c]);

        views[c]->show();
    }

    //plots
    for(std::map<PLOTWINDOW, QwtPlot*>::iterator pit = plots.begin(); pit != plots.end(); pit++)
        pit->second->show();
}

void MainWindow::updateControls()
{
    for(int i=0; i<numDebugIDs(); i++) {
        DEBUG_ID id = debugIDFromInt(i);
        layer_boxes[id]->setChecked(layer_selections[current_window][id]);
    }
}

void MainWindow::addToLayer(DEBUG_ID id, const QImage& img, float alpha)
{
    images[id].push_back(std::pair<QImage, float>(img, alpha));
}

void MainWindow::addToLayer(DEBUG_ID id, const QPointF& item, QPen pen)
{
    points[id].push_back(std::pair<QPointF, QPen>(item, pen));
}

void MainWindow::addToLayer(DEBUG_ID id, const QLineF& item, QPen pen)
{
    lines[id].push_back(std::pair<QLineF, QPen>(item, pen));
}

void MainWindow::addToLayer(DEBUG_ID id, const QRectF& item, QPen pen)
{
    rectangles[id].push_back(std::pair<QRectF, QPen>(item, pen));
}

void MainWindow::addToLayer(DEBUG_ID id, const QCircle& item, QPen pen)
{
    circles[id].push_back(std::pair<QCircle, QPen>(item, pen));
}

void MainWindow::addToLayer(DEBUG_ID id, const Polygon& item, QPen pen)
{
    polygons[id].push_back(std::pair<Polygon, QPen>(item, pen));
}

void MainWindow::addToLayer(DEBUG_ID id, const std::vector<QPointF>& items, QPen pen)
{
    BOOST_FOREACH(const QPointF& item, items) {
        addToLayer(id, item, pen);
    }
}

void MainWindow::addToLayer(DEBUG_ID id, const std::vector<QLineF>& items, QPen pen)
{
    BOOST_FOREACH(const QLineF& item, items) {
        addToLayer(id, item, pen);
    }
}

void MainWindow::addToLayer(DEBUG_ID id, const std::vector<QRectF>& items, QPen pen)
{
    BOOST_FOREACH(const QRectF& item, items) {
        addToLayer(id, item, pen);
    }
}

void MainWindow::addToLayer(DEBUG_ID id, const std::vector<QCircle>& items, QPen pen)
{
    BOOST_FOREACH(const QCircle& item, items) {
        addToLayer(id, item, pen);
    }
}

void MainWindow::addToLayer(DEBUG_ID id, const std::vector<Polygon>& items, QPen pen)
{
    BOOST_FOREACH(const Polygon& item, items) {
        addToLayer(id, item, pen);
    }
}

void MainWindow::setCurve(PLOTWINDOW win, QString name, std::vector< Vector2<double> > pts, QColor colour, QwtPlotCurve::CurveStyle style, QwtSymbol symbol)
{
    //set curve
    QVector<QPointF> qpts;
    if(curves.find(name) == curves.end()) {
        //curve does not exist - create
        curves[name] = new QwtPlotCurve(name);
    }

    BOOST_FOREACH(const Vector2<double>& p, pts) {
        qpts.push_back( QPointF(p.x, p.y) );
    }

    curves[name]->setSamples(qpts);
    curves[name]->setStyle(style);
    curves[name]->setPen(colour);
    curves[name]->setSymbol(new QwtSymbol(symbol));
    curves[name]->attach(plots[win]);

    plots[win]->replot();
}

void MainWindow::setDashedCurve(PLOTWINDOW win, QString name, std::vector<Vector2<double> > pts, QColor colour, QwtPlotCurve::CurveStyle style, QwtSymbol symbol)
{
    //set curve
    QVector<QPointF> qpts;
    if(curves.find(name) == curves.end()) {
        //curve does not exist - create
        curves[name] = new LineSegmentsCurve(name);
    }

    BOOST_FOREACH(const Vector2<double>& p, pts) {
        qpts.push_back( QPointF(p.x, p.y) );
    }

    curves[name]->setSamples(qpts);
    curves[name]->setStyle(style);
    curves[name]->setPen(colour);
    curves[name]->setSymbol(new QwtSymbol(symbol));
    curves[name]->attach(plots[win]);

    plots[win]->replot();
}

void MainWindow::setHistogram(PLOTWINDOW win, QString name, Histogram1D hist, QColor colour, QwtPlotHistogram::HistogramStyle style)
{
    //set curve
    QVector<QwtIntervalSample> samples;
    if(histograms.find(name) == histograms.end()) {
        //hist does not exist - create
        histograms[name] = new QwtPlotHistogram(name);
    }

    for(std::vector<Bin>::const_iterator b = hist.begin(); b != hist.end(); b++) {
        samples.push_back( QwtIntervalSample(b->value, b->start, b->start + b->width) );
    }

    histograms[name]->setSamples(samples);
    histograms[name]->setStyle(style);
    histograms[name]->setPen(colour);
    histograms[name]->attach(plots[win]);

    plots[win]->replot();
}

void MainWindow::toggleContinuous()
{
    m_continuous = !m_continuous;
    m_next = m_continuous;
    ui->next_pb->setEnabled(!m_continuous);
    ui->prev_pb->setEnabled(!m_continuous);
}
