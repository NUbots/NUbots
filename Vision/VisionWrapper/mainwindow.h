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

    void setFrameNo(int n);
    void resetFlags() {m_next = m_finished = false;}
    void clearLayers();
    void addToLayer(DEBUG_ID id, const QImage& img, float alpha=1);
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

    void setPlot(QString name, QwtPlotCurve *curve);
    
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
    int column_num,
        row_num;

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
    map<DEBUG_ID, vector<pair<QPointF, QColor> > > points;
    map<DEBUG_ID, vector<pair<QLineF, QColor> > > lines;
    map<DEBUG_ID, vector<pair<QRectF, QColor> > > rectangles;
    map<DEBUG_ID, vector<pair<Circle, QColor> > > circles;
    map<DEBUG_ID, vector<pair<Polygon, QColor> > > polygons;

    map<QString, pair<QwtPlot*, QwtPlotCurve*> > plots;
};

#endif // MAINWINDOW_H
