#ifndef GRAPHICSVTKWIDGET_H
#define GRAPHICSVTKWIDGET_H

#ifdef PCL_ENABLED
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/qvtk_compatibility.h>
#include <pcl/common/distances.h>

// Boost
#include <boost/math/special_functions/round.hpp>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkWorldPointPicker.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPointPicker.h>
#include <vtkCamera.h>

#include <QWidget>
#include <QMutex>
#include <QMouseEvent>
#include <QVBoxLayout>
#include <QToolBar>
#include <QStatusBar>
#include <QLabel>
#include <QEvent>

namespace Qylon{
class GraphicsVTKWidget : public PCLQVTKWidget{
    Q_OBJECT
public:
    GraphicsVTKWidget(QWidget *parent = nullptr);
    ~GraphicsVTKWidget(){}
    void setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void setScale(double factor);
    void setOriginal();

private:
    static void callbackPicked(const pcl::visualization::PointPickingEvent &event, void *arg);
    void refreshView();

    QVBoxLayout layout;
    QToolBar toolBar;
    QStatusBar statusBar;
    QLabel *currentCameraPosition;
    QLabel *currentPickedPoint;

    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentCloud;
    bool mousePressed = false;
    double currentPicked[3];
    QMutex mutex;

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    bool event(QEvent* event) override;
};
}
#endif
#endif // GRAPHICSVTKWIDGET_H
