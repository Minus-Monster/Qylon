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
    GraphicsScene(bool onVTK=false){
        isVTK = onVTK;
        if(onVTK){
#ifdef PCL_ENABLED
            VTKWidget = new GraphicsVTKWidget;
            addWidget(VTKWidget);
            VTKWidget->installEventFilter(this);
#else
            qDebug() << "VTK is not installed on your system";
#endif
        }else addItem(&Pixmap);
    }
    ~GraphicsScene(){
        deleteLater();
    }

signals:
    void currentPos(QPointF point);

private:
    bool pressed = false;
    bool isVTK = false;
    QPointF pressPoint;
    QPointF movePoint;
    QPointF releasePoint;

protected:
    bool eventFilter(QObject* obj, QEvent* event) override{
#ifdef PCL_ENABLED
        if(obj == VTKWidget) return VTKWidget->eventFilter(obj, event);
#else
        qDebug() << "VTK is not installed on your system";
#endif
        return this->eventFilter(obj, event);
    }
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override{
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
    }
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override{
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
    }
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override{
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
    }
};
}
#endif // GRAPHICSSCENE_H
