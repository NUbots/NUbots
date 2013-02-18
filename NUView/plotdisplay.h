#ifndef PLOTDISPLAY_H
#define PLOTDISPLAY_H

#include <vector>
#include <map>
#include <qwt/qwt_plot.h>
#include <qwt/qwt_symbol.h>
#include <qwt/qwt_plot_curve.h>

using std::map;
using std::vector;
using std::pair;

class PlotDisplay : public QwtPlot
{
    Q_OBJECT
public:
    explicit PlotDisplay(QWidget *parent = 0);
    virtual ~PlotDisplay();

    static vector<QString> names();
    static bool nameExists(QString name);
    static QwtSymbol getSymbol(QString name);
    static QwtPlotCurve::CurveStyle getLineStyle(QString name);
    static QColor getLineColour(QString name);

    void toggleCurve(QString name, bool enabled);
    bool isCurveEnabled(QString name);


public slots:
    static void clearMap();

    void clearCurves();
    void updateCurveData(QString name, QVector<QPointF> points);
    void updateCurveProperties(QString name, QwtSymbol symbol, QwtPlotCurve::CurveStyle lineStyle, QColor lineColour);
    virtual void replot();

private:
    void updateCurve(QString name);

signals:
    void namesUpdated(vector<QString> names);

private:
    static map<QString, QVector<QPointF> > dataMap;
    static map<QString, QwtSymbol> symbolMap;
    static map<QString, QwtPlotCurve::CurveStyle> styleMap;
    static map<QString, QColor> colourMap;

    map<QString, QwtPlotCurve*> curveMap;
    map<QString, bool> enabledCurves;
};

#endif // PLOTDISPLAY_H
