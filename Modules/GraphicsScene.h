#ifndef GRAPHICSSCENE_H
#define GRAPHICSSCENE_H
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsSceneMouseEvent>
#include <QDebug>
#ifdef PCL_ENABLED
#include "GraphicsVTKWidget.h"
#endif

namespace Qylon{
class GraphicsScene : public QGraphicsScene{
    Q_OBJECT
public:
    QGraphicsPixmapItem Pixmap;
#ifdef PCL_ENABLED
    GraphicsVTKWidget *VTKWidget;
#endif
    GraphicsScene(bool onVTK=false);
    ~GraphicsScene();
    QPointF getCurrentMousePoint(){ return movePoint; }
    void removeAllItems();

signals:
    void currentPos(QPointF point);

private:
    bool pressed = false;
    bool isVTK = false;
    QPointF pressPoint;
    QPointF movePoint;
    QPointF releasePoint;

protected:
    bool eventFilter(QObject* obj, QEvent* event) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
};
}
#endif // GRAPHICSSCENE_H
