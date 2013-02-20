#include "plotdisplay.h"
#include "qwt/qwt_symbol.h"
#include "qwt/qwt_legend.h"

map<QString, QVector<QPointF> > PlotDisplay::dataMap;
map<QString, QwtSymbol> PlotDisplay::symbolMap;
map<QString, QwtPlotCurve::CurveStyle> PlotDisplay::styleMap;
map<QString, QColor> PlotDisplay::colourMap;

PlotDisplay::PlotDisplay(QWidget *parent) :
    QwtPlot(parent)
{
    // legend
    QwtLegend *legend = new QwtLegend;
    legend->setFrameStyle(QFrame::Box|QFrame::Sunken);
    insertLegend(legend, QwtPlot::RightLegend);
    zoomer = new QwtPlotZoomer(canvas());
}

PlotDisplay::~PlotDisplay()
{
    for(map<QString, QwtPlotCurve*>::iterator cit = curveMap.begin(); cit != curveMap.end(); cit++)
        delete cit->second;
    delete zoomer;
}

vector<QString> PlotDisplay::names()
{
    vector<QString> names;
    for(map<QString, QVector<QPointF> >::const_iterator it = dataMap.begin(); it != dataMap.end(); it++)
        names.push_back(it->first);
    return names;
}

bool PlotDisplay::nameExists(QString name)
{
    map<QString, QVector<QPointF> >::iterator it = dataMap.find(name);
    return it != dataMap.end();
}

QwtSymbol PlotDisplay::getSymbol(QString name)
{
    return symbolMap[name];
}

QwtPlotCurve::CurveStyle PlotDisplay::getLineStyle(QString name)
{
    return styleMap[name];
}

QColor PlotDisplay::getLineColour(QString name)
{
    return colourMap[name];
}

void PlotDisplay::setCurveVisibility(QString name, bool visibility)
{
    visibleCurves[name] = visibility;
}

bool PlotDisplay::isCurveVisible(QString name)
{
    map<QString, bool>::iterator it = visibleCurves.find(name);
    if(it != visibleCurves.end())
        return it->second;
    else
        return false;
}

void PlotDisplay::clearMap()
{
    dataMap.clear();
}

void PlotDisplay::clearCurves()
{
    for(map<QString, QwtPlotCurve*>::iterator cit = curveMap.begin(); cit != curveMap.end(); cit++)
        delete cit->second;
    curveMap.clear();
}

void PlotDisplay::updateCurveData(QString name, QVector<QPointF> points)
{
    bool nameExisted = nameExists(name);
    dataMap[name] = points;
    map<QString, bool>::iterator it = visibleCurves.find(name);
    if(it == visibleCurves.end())
        visibleCurves[name] = false;

    updateCurve(name);

    if(!nameExisted)
        emit namesUpdated(names());
}

void PlotDisplay::updateCurveProperties(QString name, QwtSymbol symbol, QwtPlotCurve::CurveStyle lineStyle, QColor lineColour)
{
    if(nameExists(name)) {
        symbolMap[name] = symbol;
        styleMap[name] = lineStyle;
        colourMap[name] = lineColour;

        updateCurve(name);
    }
}

void PlotDisplay::updateCurve(QString name)
{
    map<QString, QwtPlotCurve*>::iterator it = curveMap.find(name);
    if(it == curveMap.end()) {
        curveMap[name] = new QwtPlotCurve(name);
    }

    QwtPlotCurve* curve = curveMap[name];

    curve->setSamples(dataMap[name]);
    curve->setStyle(styleMap[name]);
    curve->setSymbol(new QwtSymbol(symbolMap[name]));
    curve->setPen(QPen(colourMap[name]));

    replot();
}

void PlotDisplay::replot()
{
    map<QString, QwtPlotCurve*>::iterator cit;
    for(cit = curveMap.begin(); cit != curveMap.end(); cit++) {
        if(visibleCurves[cit->first])
            cit->second->attach(this);
        else
            cit->second->detach();
    }
    QwtPlot::replot();
}
