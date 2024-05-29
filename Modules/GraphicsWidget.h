#ifndef GRAPHICSWIDGET_H
#define GRAPHICSWIDGET_H
#include <QWidget>
#include <QSpacerItem>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QToolBar>
#include <QFileDialog>
#include <QStatusBar>
#include <QMessageBox>
#include <QImageReader>
#include <QTimer>
#include "GraphicsView.h"
#include "GraphicsScene.h"

namespace Qylon{
class GraphicsWidgetSettings : public QDialog{
    Q_OBJECT
public:
};

class GraphicsWidget : public QWidget{
    Q_OBJECT
public:
    GraphicsWidget(QWidget *parent = nullptr);
    ~GraphicsWidget();
    void initialize(bool isVTK=false);
    void setToolBarEnable(bool on);

signals:
    void updateWidget();

public slots:
    void setImage(const QImage image);
    void setFit(bool on);
    void setTimerEnable(bool on);
#ifdef PCL_ENABLED
    void setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudData, QString pointCloudName);
#endif

private:
    GraphicsView *view;
    GraphicsScene *scene;
    QLabel labelCoordinate;
    QLabel labelPixelColor;
    QLabel labelImageInformation;
    QLineEdit lineEditPixelColor;
    QToolBar toolBar;
    QStatusBar statusBar;
    QImage currentImage;
    double currentRatioValue = 100.;

    QLabel labelFPS;
    QTimer fpsTimer;
    int timerCnt = 0;


    QVBoxLayout layout;
};

}
#endif // GRAPHICSWIDGET_H
