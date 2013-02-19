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
#include <qwt/qwt_plot_magnifier.h>
#include <qwt/qwt_plot_zoomer.h>

#include <map>
#include <vector>

#include "Vision/basicvisiontypes.h"

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
        p3 = 2
    };

    PLOTWINDOW winFromInt(int i) {
        switch(i) {
        case 1: return p2;
        case 2: return p3;
        default: return p1;
        }
    }

    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void setFrameNo(int n);
    void resetFlags() {m_next = m_finished = false;}
    void clearLayers();
    void addToLayer(DEBUG_ID id, const QImage& img, float alpha=1);
    void addToLayer(DEBUG_ID id, const QPointF& item, QPen pen);
    void addToLayer(DEBUG_ID id, const QLineF& item, QPen pen);
    void addToLayer(DEBUG_ID id, const QRectF& item, QPen pen);
    void addToLayer(DEBUG_ID id, const QCircle& item, QPen pen);
    void addToLayer(DEBUG_ID id, const Polygon& item, QPen pen);

    void addToLayer(DEBUG_ID id, const vector<QPointF>& items, QPen pen);
    void addToLayer(DEBUG_ID id, const vector<QLineF>& items, QPen pen);
    void addToLayer(DEBUG_ID id, const vector<QRectF>& items, QPen pen);
    void addToLayer(DEBUG_ID id, const vector<QCircle>& items, QPen pen);
    void addToLayer(DEBUG_ID id, const vector<Polygon>& items, QPen pen);

    void setPlot(PLOTWINDOW win, QString name, vector<Point> pts, QColor colour, QwtPlotCurve::CurveStyle style);
    
    bool finished() const {return m_finished;}
    bool next() const {return m_next;}

public slots:
    void refresh();

private slots:
    void setFinished() {m_finished = true;}
    void setNext() {m_next = true;}

private:
    bool m_finished,
         m_next;

    Ui::MainWindow* ui;
    QWidgetList windows;

    //layout items
    QVBoxLayout* layers_layout;
    QWidget* layers_box;

    QGridLayout *windows_layout;
    QWidget *windows_box;
    //ui->gridLayout->addLayout(gridEPG, 1, 0, 1, 3);

    QGraphicsView* view;
    QGraphicsScene* scene;
    QImage* canvas;
    map<DEBUG_ID, QCheckBox*> layer_selections;
    map<DEBUG_ID, vector<pair<QImage, float> > > images;
    map<DEBUG_ID, vector<pair<QPointF, QPen> > > points;
    map<DEBUG_ID, vector<pair<QLineF, QPen> > > lines;
    map<DEBUG_ID, vector<pair<QRectF, QPen> > > rectangles;
    map<DEBUG_ID, vector<pair<QCircle, QPen> > > circles;
    map<DEBUG_ID, vector<pair<Polygon, QPen> > > polygons;

    map<QString, QwtPlotCurve*> curves;
    map<PLOTWINDOW, QwtPlot*> plots;
    //map<PLOTWINDOW, QwtPlotMagnifier*> magnifiers;
    map<PLOTWINDOW, QwtPlotZoomer*> zoomers;
};

#endif // MAINWINDOW_H
