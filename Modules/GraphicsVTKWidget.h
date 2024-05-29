#ifndef GRAPHICSVTKWIDGET_H
#define GRAPHICSVTKWIDGET_H

#include <QObject>
#include <QDebug>
#include <QMouseEvent>
#include <QWheelEvent>

#ifdef PCL_ENABLED
#include <QVTKOpenGLNativeWidget.h>
#include <QVTKRenderWidget.h>

#include <vtkGenericOpenGLRenderWindow.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#endif

namespace Qylon{
#ifdef PCL_ENABLED
class GraphicsVTKWidget : public QVTKOpenGLNativeWidget{
//class GraphicsVTKWidget : public QVTKRenderWidget{
    Q_OBJECT
public:
    GraphicsVTKWidget();

    void setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudData, QString pointCloudName);
    void setViewUp();
    void setViewDown();
    void setViewLeft();
    void setViewRight();
    void setScale(float scale);
    void resetScale();
    void updateCameraPosition();
    static void pickingPointCallBack(const pcl::visualization::PointPickingEvent &event);
    static void keyboardCallBack(const pcl::visualization::KeyboardEvent& event, void* viewer_void) {}

    static void mouseCallBack(const pcl::visualization::MouseEvent& event, void* viewer_void);
//    bool eventFilter(QObject *obj, QEvent *event) override{
//        if(event->type() == QEvent::PolishRequest || event->type() == QEvent::UpdateRequest || event->type() == QEvent::Paint) return false;

//        if(event->type() == QEvent::Wheel){
//            QWheelEvent *wheel     = static_cast<QWheelEvent*>(event);
//            if(wheel->angleDelta().y() > 0) this->interactor()->MouseWheelForwardEvent();
//            else if(wheel->angleDelta().y() < 0) this->interactor()->MouseWheelBackwardEvent();
//            return false;
//        }
//        if(event->type() == QEvent::GraphicsSceneMouseMove) this->interactor()->MouseMoveEvent();
//        if(event->type() == QEvent::GraphicsSceneMousePress) this->interactor()->LeftButtonPressEvent();
//        if(event->type() == QEvent::GraphicsSceneMouseRelease) this->interactor()->LeftButtonReleaseEvent();
//        if(event->type() == QEvent::GraphicsSceneContextMenu) this->interactor()->RightButtonPressEvent();

//        updateCameraPosition();
//        return false;
//    }
    pcl::visualization::PCLVisualizer::Ptr viewer;

signals:
    void currentCameraPosition(QString text);

private:
    pcl::visualization::Camera defaultCamera;
};
#endif

}
#endif // GRAPHICSVTKWIDGET_H
