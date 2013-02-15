#include "plotdisplay.h"
#include "qwt/qwt_symbol.h"

PlotDisplay::PlotDisplay(QWidget *parent) :
    QwtPlot(parent)
{
}

PlotDisplay::~PlotDisplay()
{
    //clear();
}

void PlotDisplay::clear()
{
    map<QString, QwtPlotCurve*>::iterator mit;;
    for(mit = curveMap.begin(); mit != curveMap.end(); mit++)
        delete mit->second;
    curveMap.clear();
}

void PlotDisplay::updateCurve(const QwtPlotCurve* curve, QString name)
{
    map<QString, QwtPlotCurve*>::iterator mit = curveMap.find(name);
    if(mit == curveMap.end()) {
        //non-existant element, create
        curveMap[name] = new QwtPlotCurve(name);
    }

    QwtPlotCurve* c = curveMap[name];
    c->setBrush(curve->brush());
    c->setStyle(curve->style());
    const QwtSymbol* s = curve->symbol();
    if(s)
        c->setSymbol(new QwtSymbol(*s));

    const QwtSeriesData<QPointF>* data = curve->data();
    QVector<QPointF> pts;
    if(data) {
        for(size_t i=0; i < data->size(); i++) {
            pts.push_back(data->sample(i));
        }
    }

    c->setSamples(pts);

    replot();
}

void PlotDisplay::replot()
{
    map<QString, QwtPlotCurve*>::iterator mit;;
    for(mit = curveMap.begin(); mit != curveMap.end(); mit++)
        mit->second->attach(this);
    QwtPlot::replot();
}
