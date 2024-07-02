#include "GraphicsScene.h"

Qylon::GraphicsScene::GraphicsScene(bool onVTK) : isVTK(onVTK){
    if(onVTK){
#ifdef PCL_ENABLED
        VTKWidget = new GraphicsVTKWidget;
        addWidget(VTKWidget);
        VTKWidget->installEventFilter(this);
#else
        qDebug() << "VTK is not installed on your system";
#endif
    }else{
        Pixmap.setZValue(-1);
        addItem(&Pixmap);
    }
}

Qylon::GraphicsScene::~GraphicsScene(){
#ifdef PCL_ENABLED
    delete VTKWidget;
#endif
}

bool Qylon::GraphicsScene::eventFilter(QObject *obj, QEvent *event){
#ifdef PCL_ENABLED
    if(obj == VTKWidget) return VTKWidget->eventFilter(obj, event);
#else
    qDebug() << "VTK is not installed on your system";
#endif
    return QGraphicsScene::eventFilter(obj, event);
}

void Qylon::GraphicsScene::mouseMoveEvent(QGraphicsSceneMouseEvent *event){
    if(isVTK){
#ifdef PCL_ENABLED
        VTKWidget->eventFilter(VTKWidget, event);
#else
        qDebug() << "VTK is not installed on your system";
#endif
        return;
    }
    if(event->scenePos().toPoint().x() < 0) return;
    if(event->scenePos().toPoint().y() < 0 ) return;
    if(event->scenePos().toPoint().x() >= sceneRect().width()) return;
    if(event->scenePos().toPoint().y() >= sceneRect().height()) return;

    emit currentPos(event->scenePos());
    movePoint = event->scenePos();
    QGraphicsScene::mouseMoveEvent(event);
}

void Qylon::GraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent *event){
    if(isVTK){
#ifdef PCL_ENABLED
        VTKWidget->eventFilter(VTKWidget, event);
#else
        qDebug() << "VTK is not installed on your system";
#endif
        return;
    }
    if(event->scenePos().toPoint().x() < 0) return;
    if(event->scenePos().toPoint().y() < 0 ) return;
    if(event->scenePos().toPoint().x() >= sceneRect().width()) return;
    if(event->scenePos().toPoint().y() >= sceneRect().height()) return;


    pressPoint = event->scenePos();
    pressed = true;
    QGraphicsScene::mousePressEvent(event);
}

void Qylon::GraphicsScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    if(isVTK){
#ifdef PCL_ENABLED
        VTKWidget->eventFilter(VTKWidget, event);
#else
        qDebug() << "VTK is not installed on your system";
#endif
        return;
    }
    if(event->scenePos().toPoint().x() < 0) return;
    if(event->scenePos().toPoint().y() < 0 ) return;
    if(event->scenePos().toPoint().x() >= sceneRect().width()) return;
    if(event->scenePos().toPoint().y() >= sceneRect().height()) return;

    pressed = false;
    QGraphicsScene::mouseReleaseEvent(event);
}
