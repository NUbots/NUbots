#ifndef PLOTDISPLAY_H
#define PLOTDISPLAY_H

#include <map>
#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_curve.h>

using std::map;

class PlotDisplay : public QwtPlot
{
    Q_OBJECT
public:
    explicit PlotDisplay(QWidget *parent = 0);
    virtual ~PlotDisplay();
    
signals:
    
public slots:
    void clear();
    void updateCurve(const QwtPlotCurve *curve, QString name);
    virtual void replot();
    
private:
    map<QString, QwtPlotCurve*> curve_map;
};

#endif // PLOTDISPLAY_H
