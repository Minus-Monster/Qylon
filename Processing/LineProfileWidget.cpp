#include "LineProfileWidget.h"
#include "Modules/GraphicsView.h"

Qylon::RulerItem::RulerItem(const QLineF line, GraphicsView *parent) : QGraphicsLineItem(line), parent(parent){
    setAcceptHoverEvents(true);
    setPen(Qt::NoPen);
}

void Qylon::RulerItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    painter->save();
    QLineF line = this->line();
    double angle = std::atan2(line.dy(), line.dx());

    QTransform transform = parent->transform();
    qreal scale = std::sqrt(transform.m11()*transform.m11() + transform.m22()*transform.m22());
    qreal tick = 20 / scale*0.9;
    int outlineTick = 12/ scale*0.9;
    int inlineTick = 7/ scale*0.9;

    if(outlineTick <= 1) outlineTick = 2;
    if(inlineTick <= 1) inlineTick = 1;
    if(tick <= 4) tick = 4;

    QPointF offset(sin(angle) * tick / 2, -cos(angle) * tick / 2);
    QPointF topLeft = line.p1() - offset;
    QPointF topRight = line.p1() + offset;
    QPointF bottomLeft = line.p2() - offset;
    QPointF bottomRight = line.p2() + offset;

    painter->setPen(QPen(Qt::white, outlineTick));
    painter->drawLine(topLeft, topRight);
    painter->drawLine(bottomLeft, bottomRight);
    painter->drawLine(line.p1(), line.p2());

    painter->setPen(QPen(QColor(20,72,126), inlineTick));
    painter->drawLine(topLeft, topRight);
    painter->drawLine(bottomLeft, bottomRight);
    painter->drawLine(line.p1(), line.p2());
    painter->restore();
}
