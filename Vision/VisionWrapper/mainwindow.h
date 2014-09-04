#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QCheckBox>
#include <QGraphicsView>
#include <QImage>
#include <QGraphicsPixmapItem>
#include <QWidget>
#include <QVBoxLayout>
#include <QGridLayout>

#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_histogram.h>
#include <qwt_symbol.h>
#include <qwt/qwt_plot_magnifier.h>
#include <qwt/qwt_plot_zoomer.h>

#include <map>
#include <vector>

#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/histogram1d.h"

using std::map;
using std::vector;
using std::pair;
using namespace Vision;

namespace Vision {
class QCircle {
public:
    QCircle(QPointF centre, double radius) {m_centre = centre; m_radius = radius;}

    QPointF m_centre;
    double m_radius;
};
class Polygon {
public:
    Polygon(const QPolygonF& polygon, bool filled = false)
    {
        m_polygon = polygon;
        m_filled = filled;
    }

    QPolygonF m_polygon;
    bool m_filled;
};
}

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    enum PLOTWINDOW {
        p1 = 0,
        p2 = 1,
        p3 = 2,
        p4 = 3
    };

    PLOTWINDOW winFromInt(int i) {
        switch(i) {
        case 1: return p2;
        case 2: return p3;
        case 3: return p4;
        default: return p1;
        }
    }

    static const size_t NUM_PLOTS = 4;
    static const size_t NUM_CANVASES = 2;

    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void setFrameNo(int n);
    void resetFlags();
    void clearLayers();
    void addToLayer(DEBUG_ID id, const QImage& img, float alpha=1);
    void addToLayer(DEBUG_ID id, const QPointF& item, QPen pen);
    void addToLayer(DEBUG_ID id, const QLineF& item, QPen pen);
    void addToLayer(DEBUG_ID id, const QRectF& item, QPen pen);
    void addToLayer(DEBUG_ID id, const QCircle& item, QPen pen);
    void addToLayer(DEBUG_ID id, const Polygon& item, QPen pen);

    void addToLayer(DEBUG_ID id, const std::vector<QPointF>& items, QPen pen);
    void addToLayer(DEBUG_ID id, const std::vector<QLineF>& items, QPen pen);
    void addToLayer(DEBUG_ID id, const std::vector<QRectF>& items, QPen pen);
    void addToLayer(DEBUG_ID id, const std::vector<QCircle>& items, QPen pen);
    void addToLayer(DEBUG_ID id, const std::vector<Polygon>& items, QPen pen);

    void setCurve(PLOTWINDOW win, QString name, std::vector<Vector2<double> > pts, QColor colour, QwtPlotCurve::CurveStyle style, QwtSymbol symbol = QwtSymbol());
    void setDashedCurve(PLOTWINDOW win, QString name, std::vector<Vector2<double> > pts, QColor colour, QwtPlotCurve::CurveStyle style, QwtSymbol symbol = QwtSymbol());
    void setHistogram(PLOTWINDOW win, QString name, Histogram1D hist, QColor colour, QwtPlotHistogram::HistogramStyle style);
    
    bool finished() const {return m_finished;}
    bool next() const {return m_next;}
    bool prev() const {return m_prev;}

public slots:
    void refresh();

private slots:
    void setFinished() {m_finished = true;}
    void setNext() {m_next = true;}
    void setPrev() {m_prev = true;}
    void toggleContinuous();
    void setWindow(int w) { current_window = w; updateControls();}

private:
    void updateControls();

private:
    bool m_finished,
         m_continuous,
         m_next,
         m_prev;

    Ui::MainWindow* ui;
    QWidgetList windows;

    //layout items
    QVBoxLayout* layers_layout;
    QWidget* layers_box;

    QGridLayout *windows_layout;
    QWidget *windows_box;
    //ui->gridLayout->addLayout(gridEPG, 1, 0, 1, 3);

    std::vector<QGraphicsView*> views;
    std::vector<QGraphicsScene*> scenes;
    std::vector<QImage*> canvases;
    std::vector< map<DEBUG_ID, bool> > layer_selections;
    //QImage* canvas;

    map<DEBUG_ID, QCheckBox*> layer_boxes;

    size_t current_window;

    map<DEBUG_ID, std::vector<std::pair<QImage, float> > > images;
    map<DEBUG_ID, std::vector<std::pair<QPointF, QPen> > > points;
    map<DEBUG_ID, std::vector<std::pair<QLineF, QPen> > > lines;
    map<DEBUG_ID, std::vector<std::pair<QRectF, QPen> > > rectangles;
    map<DEBUG_ID, std::vector<std::pair<QCircle, QPen> > > circles;
    map<DEBUG_ID, std::vector<std::pair<Polygon, QPen> > > polygons;

    map<QString, QwtPlotCurve*> curves;
    map<QString, QwtPlotHistogram*> histograms;
    map<PLOTWINDOW, QwtPlot*> plots;
    //map<PLOTWINDOW, QwtPlotMagnifier*> magnifiers;
    map<PLOTWINDOW, QwtPlotZoomer*> zoomers;
};

#endif // MAINWINDOW_H
