#include "linesegmentscurve.h"

LineSegmentsCurve::LineSegmentsCurve(const QString &title) : QwtPlotCurve(title)
{

}

void LineSegmentsCurve::drawCurve( QPainter *painter, int style, const QwtScaleMap &xMap, const QwtScaleMap &yMap, const QRectF &canvasRect, int from, int to ) const
{
    for (int i = from; i <= to; i++){
        //only draw line between each pair,
        if(i%2 == 1) {
            //drawLines(painter, xMap, yMap, canvasRect, preceding_from, i>from ? i-1 : i); // or drawLines, drawSticks, drawDots
            drawLines(painter, xMap, yMap, canvasRect, i-1, i);
        }
    }
}
