#ifndef GRAPHICSSCENE_H
#define GRAPHICSSCENE_H
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsSceneMouseEvent>
#include <QDebug>


namespace Qylon{
class GraphicsScene : public QGraphicsScene{
    Q_OBJECT
public:
    QGraphicsPixmapItem Pixmap;
    GraphicsScene();
    ~GraphicsScene();
    QPointF getCurrentMousePoint(){ return movePoint; }

signals:
    void currentPos(QPointF point);

private:
    bool pressed = false;
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
