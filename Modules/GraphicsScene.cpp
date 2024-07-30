#include "GraphicsScene.h"

Qylon::GraphicsScene::GraphicsScene(){
    Pixmap.setZValue(-1);
    addItem(&Pixmap);
}

Qylon::GraphicsScene::~GraphicsScene(){
}

bool Qylon::GraphicsScene::eventFilter(QObject *obj, QEvent *event){
    return QGraphicsScene::eventFilter(obj, event);
}

void Qylon::GraphicsScene::mouseMoveEvent(QGraphicsSceneMouseEvent *event){
    if(event->scenePos().toPoint().x() < 0) return;
    if(event->scenePos().toPoint().y() < 0 ) return;
    if(event->scenePos().toPoint().x() >= sceneRect().width()) return;
    if(event->scenePos().toPoint().y() >= sceneRect().height()) return;

    emit currentPos(event->scenePos());
    movePoint = event->scenePos();
    QGraphicsScene::mouseMoveEvent(event);
}

void Qylon::GraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent *event){
    if(event->scenePos().toPoint().x() < 0) return;
    if(event->scenePos().toPoint().y() < 0 ) return;
    if(event->scenePos().toPoint().x() >= sceneRect().width()) return;
    if(event->scenePos().toPoint().y() >= sceneRect().height()) return;


    pressPoint = event->scenePos();
    pressed = true;
    QGraphicsScene::mousePressEvent(event);
}

void Qylon::GraphicsScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    if(event->scenePos().toPoint().x() < 0) return;
    if(event->scenePos().toPoint().y() < 0 ) return;
    if(event->scenePos().toPoint().x() >= sceneRect().width()) return;
    if(event->scenePos().toPoint().y() >= sceneRect().height()) return;

    pressed = false;
    QGraphicsScene::mouseReleaseEvent(event);
}
