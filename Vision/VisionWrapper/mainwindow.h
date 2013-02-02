#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QCheckBox>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>

#include <map>
#include <vector>

#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/Interfaces/renderable.h"

using std::map;
using std::vector;
using std::pair;
using namespace Vision;

namespace Vision {
class Circle {
public:
    Circle(QPointF centre, double radius) {m_centre = centre; m_radius = radius;}

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
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void clearLayers();
    void refresh();
    void addToLayer(DEBUG_ID id, const QPointF& item, QColor colour);
    void addToLayer(DEBUG_ID id, const QLineF& item, QColor colour);
    void addToLayer(DEBUG_ID id, const QRectF& item, QColor colour);
    void addToLayer(DEBUG_ID id, const Circle& item, QColor colour);
    void addToLayer(DEBUG_ID id, const Polygon& item, QColor colour);

    void addToLayer(DEBUG_ID id, const vector<QPointF>& items, QColor colour);
    void addToLayer(DEBUG_ID id, const vector<QLineF>& items, QColor colour);
    void addToLayer(DEBUG_ID id, const vector<QRectF>& items, QColor colour);
    void addToLayer(DEBUG_ID id, const vector<Circle>& items, QColor colour);
    void addToLayer(DEBUG_ID id, const vector<Polygon>& items, QColor colour);

    void setPlot(QString name, QwtPlotCurve curve);
    void setBaseImage(const QImage& img) {*base_image = img;}

    void addLayerOption(QString name);
    
private:
    Ui::MainWindow *ui;
    QWidgetList windows;

    QImage* image;
    QImage* base_image;
    map<DEBUG_ID, QCheckBox*> layer_selections;
    map<DEBUG_ID, vector<pair<QPointF, QColor> > > points;
    map<DEBUG_ID, vector<pair<QLineF, QColor> > > lines;
    map<DEBUG_ID, vector<pair<QRectF, QColor> > > rectangles;
    map<DEBUG_ID, vector<pair<Circle, QColor> > > circles;
    map<DEBUG_ID, vector<pair<Polygon, QColor> > > polygons;

    map<QString, QwtPlot*> plots;
};

#endif // MAINWINDOW_H
