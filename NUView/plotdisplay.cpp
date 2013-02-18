#include "plotdisplay.h"
#include "qwt/qwt_symbol.h"

set<QString> PlotDisplay::curveNames;

PlotDisplay::PlotDisplay(QWidget *parent) :
    QwtPlot(parent)
{
}

PlotDisplay::~PlotDisplay()
{
    //clear();
}

void PlotDisplay::updateNames(QString name)
{
    curveNames.insert(name);
}

void PlotDisplay::clearMap()
{
    map<QString, pair<bool, QwtPlotCurve*> >::iterator mit;;
    for(mit = curveMap.begin(); mit != curveMap.end(); mit++)
        delete mit->second.second;
    curveMap.clear();
}

void PlotDisplay::updateCurve(QVector<QPointF> points, QString name)
{
    updateNames(name);
    map<QString, pair<bool, QwtPlotCurve*> >::iterator mit = curveMap.find(name);
    if(mit == curveMap.end()) {
        //non-existant element, create
        curveMap[name] = pair<bool, QwtPlotCurve*>(false, new QwtPlotCurve(name));   //default to disabled
    }

    QwtPlotCurve* c = curveMap[name].second;
    c->setSamples(points);
    replot();
}

void PlotDisplay::replot()
{
    map<QString, pair<bool, QwtPlotCurve*> >::iterator mit;
    for(mit = curveMap.begin(); mit != curveMap.end(); mit++) {
        if(mit->second.first)
            mit->second.second->attach(this);
        else
            mit->second.second->detach();
    }
    QwtPlot::replot();
}
