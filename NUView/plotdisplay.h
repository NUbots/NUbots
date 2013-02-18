#ifndef PLOTDISPLAY_H
#define PLOTDISPLAY_H

#include <set>
#include <map>
#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_curve.h>

using std::map;
using std::set;
using std::pair;

class PlotDisplay : public QwtPlot
{
    Q_OBJECT

public:
    static set<QString> curveNames;
    map<QString, pair<bool, QwtPlotCurve*> > curveMap;

public:
    explicit PlotDisplay(QWidget *parent = 0);
    virtual ~PlotDisplay();
    static void updateNames(QString name);
    
public slots:
    static void clearNames() {curveNames.clear();}
    void clearMap();
    void updateCurve(QVector<QPointF> points, QString name);
    virtual void replot();
};

#endif // PLOTDISPLAY_H
