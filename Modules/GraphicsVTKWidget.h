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
    WidgetInformation() : treeWidget(new QTreeWidget),
        itemCurrentPoint(new QTreeWidgetItem()),
        itemPos(new QTreeWidgetItem),
        itemFocal(new QTreeWidgetItem()),
        itemView(new QTreeWidgetItem()),
        itemCamera(new QTreeWidgetItem)    {
        setLayout(&layout);
        layout.addWidget(treeWidget);
        treeWidget->setHeaderLabels(QStringList() << "Feature" << "Values" << "Values" << "Values");

        // Set item labels
        itemCurrentPoint->setText(0, "Picked");
        itemPos->setText(0, "Position");
        itemFocal->setText(0, "Focal Point");
        itemView->setText(0, "View Direction");

        itemCamera->setText(0, "F.O.V");
        itemCamera->addChild(itemPos);
        itemCamera->addChild(itemFocal);
        itemCamera->addChild(itemView);

        // Add items to the tree widget
        treeWidget->addTopLevelItem(itemCamera);
        treeWidget->addTopLevelItem(itemCurrentPoint);
        treeWidget->expandItem(itemCamera);
    }

public slots:
    void setCameraPosition(double p_x, double p_y, double p_z,
                           double f_x, double f_y, double f_z,
                           double v_x, double v_y, double v_z) {
        // Update Position item
        itemPos->setText(1, QString::number(p_x));
        itemPos->setText(2, QString::number(p_y));
        itemPos->setText(3, QString::number(p_z));

        // Update Focal Point item
        itemFocal->setText(1, QString::number(f_x));
        itemFocal->setText(2, QString::number(f_y));
        itemFocal->setText(3, QString::number(f_z));

        // Update View Direction item
        itemView->setText(1, QString::number(v_x));
        itemView->setText(2, QString::number(v_y));
        itemView->setText(3, QString::number(v_z));
    }

    void setCurrentPoint(double x, double y, double z) {
        // Update Current Point item
        itemCurrentPoint->setText(1, QString::number(x));
        itemCurrentPoint->setText(2, QString::number(y));
        itemCurrentPoint->setText(3, QString::number(z));
    }

private:
    QVBoxLayout layout;
    QTreeWidget *treeWidget;
    QTreeWidgetItem *itemCamera;
    QTreeWidgetItem *itemCurrentPoint;
    QTreeWidgetItem *itemPos;
    QTreeWidgetItem *itemFocal;
    QTreeWidgetItem *itemView;
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
    static void callbackPicked(const pcl::visualization::PointPickingEvent &event, void *arg);
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
    /*
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    */
};
}
#endif
#endif // GRAPHICSVTKWIDGET_H
