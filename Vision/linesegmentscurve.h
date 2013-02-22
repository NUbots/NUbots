#ifndef LINESEGMENTSCURVE_H
#define LINESEGMENTSCURVE_H

#include "qwt/qwt_plot_curve.h"

class LineSegmentsCurve : public QwtPlotCurve
{
public:
    LineSegmentsCurve( const QString &title = QString::null );

protected:
    virtual void drawCurve( QPainter *p, int style, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const;
};

#endif // LINESEGMENTSCURVE_H
