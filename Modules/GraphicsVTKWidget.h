#ifndef GRAPHICSVTKWIDGET_H
#define GRAPHICSVTKWIDGET_H

#ifdef PCL_ENABLED
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/qvtk_compatibility.h>
// Boost
#include <boost/math/special_functions/round.hpp>
#include <vtkGenericOpenGLRenderWindow.h>
#include <QWidget>
#include <QMutex>

namespace Qylon{
class GraphicsVTKWidget : public PCLQVTKWidget{
    Q_OBJECT
public:
    GraphicsVTKWidget(QWidget *parent = nullptr);
    ~GraphicsVTKWidget(){}
    void setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
private:
    void refreshView();

protected:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    bool isAdded =false;
    QMutex mutex;
};
}
#endif
#endif // GRAPHICSVTKWIDGET_H
