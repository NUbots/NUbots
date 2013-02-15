#ifndef PLOTDISPLAY_H
#define PLOTDISPLAY_H

#include <set>
#include <map>
#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_curve.h>

using std::map;
using std::set;

class PlotDisplay : public QwtPlot
{
    Q_OBJECT

public:
    static set<QString> curveNames;
    map<QString, QwtPlotCurve*> curveMap;
    map<QString, bool> curvesEnabled;

public:
    explicit PlotDisplay(QWidget *parent = 0);
    virtual ~PlotDisplay();
    static void updateNames(QString name);

signals:
    
public slots:
    void clear();
    void updateCurve(const QwtPlotCurve *curve, QString name);
    virtual void replot();
};

#endif // PLOTDISPLAY_H
