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
#include <QTreeWidget>

namespace Qylon{
class WidgetInformation : public QWidget{
    Q_OBJECT
public:
    WidgetInformation() : treeWidget(new QTreeWidget){
        setLayout(&layout);
        layout.addWidget(treeWidget);
        treeWidget->setHeaderLabels(QStringList() << "Feature" << "Value");

        // Set item labels
        itemXCoordinate.setText(0, "X Coordinate");
        itemYCoordinate.setText(0, "Y Coordinate");
        itemZCoordinate.setText(0, "Z Coordinate");
        itemDistance.setText(0, "Distance");

        itemPos.setText(0, "Position");
        itemFocal.setText(0, "Focal Point");
        itemView.setText(0, "View Direction");

        itemPos.addChild(&itemPosX);
        itemPos.addChild(&itemPosY);
        itemPos.addChild(&itemPosZ);
        itemPosX.setText(0, "X");
        itemPosY.setText(0, "Y");
        itemPosZ.setText(0, "Z");

        itemFocal.addChild(&itemFocalX);
        itemFocal.addChild(&itemFocalY);
        itemFocal.addChild(&itemFocalZ);
        itemFocalX.setText(0, "X");
        itemFocalY.setText(0, "Y");
        itemFocalZ.setText(0, "Z");

        itemView.addChild(&itemViewX);
        itemView.addChild(&itemViewY);
        itemView.addChild(&itemViewZ);
        itemViewX.setText(0, "X");
        itemViewY.setText(0, "Y");
        itemViewZ.setText(0, "Z");

        treeWidget->addTopLevelItem(&itemPos);
        treeWidget->addTopLevelItem(&itemFocal);
        treeWidget->addTopLevelItem(&itemView);
        treeWidget->addTopLevelItem(&itemXCoordinate);
        treeWidget->addTopLevelItem(&itemYCoordinate);
        treeWidget->addTopLevelItem(&itemZCoordinate);
        treeWidget->addTopLevelItem(&itemDistance);
    }

public slots:
    void setCameraPosition(double p_x, double p_y, double p_z,
                           double f_x, double f_y, double f_z,
                           double v_x, double v_y, double v_z) {
        itemPosX.setText(1, QString::number(p_x));
        itemPosY.setText(1, QString::number(p_y));
        itemPosZ.setText(1, QString::number(p_z));

        itemFocalX.setText(1, QString::number(f_x));
        itemFocalY.setText(1, QString::number(f_y));
        itemFocalZ.setText(1, QString::number(f_z));

        itemViewX.setText(1, QString::number(v_x));
        itemViewY.setText(1, QString::number(v_y));
        itemViewZ.setText(1, QString::number(v_z));
    }

    void setCurrentPoint(double x, double y, double z) {
        // Update Current Point item
        itemXCoordinate.setText(1, QString::number(x) + " mm");
        itemYCoordinate.setText(1, QString::number(y) + " mm");
        itemZCoordinate.setText(1, QString::number(z) + " mm");
        itemDistance.setText(1, QString::number(sqrt(x*x+y*y+z*z)) + " mm");
    }

private:
    QVBoxLayout layout;
    QTreeWidget *treeWidget;
    QTreeWidgetItem itemXCoordinate;
    QTreeWidgetItem itemYCoordinate;
    QTreeWidgetItem itemZCoordinate;
    QTreeWidgetItem itemDistance;

    QTreeWidgetItem itemPos;
    QTreeWidgetItem itemPosX;
    QTreeWidgetItem itemPosY;
    QTreeWidgetItem itemPosZ;
    QTreeWidgetItem itemFocal;
    QTreeWidgetItem itemFocalX;
    QTreeWidgetItem itemFocalY;
    QTreeWidgetItem itemFocalZ;
    QTreeWidgetItem itemView;
    QTreeWidgetItem itemViewX;
    QTreeWidgetItem itemViewY;
    QTreeWidgetItem itemViewZ;
};
class GraphicsVTKWidget : public PCLQVTKWidget{
    Q_OBJECT
public:
    GraphicsVTKWidget(QWidget *parent = nullptr);
    ~GraphicsVTKWidget(){}
    void setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void setScale(double factor);
    void setOriginal();
    void setCameraPosition(double p_x, double p_y, double p_z,
                           double f_x, double f_y, double f_z,
                           double v_x, double v_y, double v_z);
    bool loadPointCloud(QString file);
    bool savePointCloud(QString file);
    QWidget* getInformationWidget(){ return infoWidget; }

signals:
    void sendCameraPosition(double p_x, double p_y, double p_z,
                            double f_x, double f_y, double f_z,
                            double v_x, double v_y, double v_z);
    void sendCurrentPoint(double x, double y, double z);

private:
    static void callbackMouse(const pcl::visualization::MouseEvent &event, void *arg);
    static void callbackAreaPicked(const pcl::visualization::AreaPickingEvent &event, void *arg);
    void refreshView();

    QVBoxLayout layout;
    QToolBar toolBar;
    QStatusBar statusBar;
    QLabel *currentMousePoint;

    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentCloud;
    bool mousePressed = false;
    double currentPicked[3];
    QMutex mutex;

    WidgetInformation *infoWidget;

protected:
    bool event(QEvent* event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    /*
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    */
};
}
#endif
#endif // GRAPHICSVTKWIDGET_H
