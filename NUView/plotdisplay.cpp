#include "plotdisplay.h"
#include "qwt/qwt_symbol.h"

PlotDisplay::PlotDisplay(QWidget *parent) :
    QwtPlot(parent)
{
}

PlotDisplay::~PlotDisplay()
{
    map<QString, QwtPlotCurve*>::iterator mit;;
    for(mit = curve_map.begin(); mit != curve_map.end(); mit++)
        delete mit->second;
}

void PlotDisplay::updateCurve(const QwtPlotCurve* curve, QString name)
{
    map<QString, QwtPlotCurve*>::iterator mit = curve_map.find(name);
    if(mit == curve_map.end()) {
        //non-existant element, create
        curve_map[name] = new QwtPlotCurve(name);
    }

    QwtPlotCurve* c = curve_map[name];
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
}
