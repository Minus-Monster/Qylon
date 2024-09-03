#ifdef PCL_ENABLED
#include "GraphicsVTKWidget.h"
#include <vtkArrowSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkTransform.h>
#include <vtkSmartPointer.h>
#include <pcl/common/common.h>
#include <QFileDialog>

Qylon::GraphicsVTKWidget::GraphicsVTKWidget(QWidget *parent) : infoWidget(new WidgetInformation){
    setLayout(&layout);
    layout.addWidget(&toolBar);
    layout.addWidget(&statusBar,0, Qt::AlignBottom);

    currentMousePoint = new QLabel;
    statusBar.addWidget(currentMousePoint);

    QList<QAction*> actions;
    QAction *loadImage = new QAction(QIcon(":/Resources/Icon/icons8-opened-folder-48.png"),"");
    connect(loadImage, &QAction::triggered, this, [=]{
        auto file = QFileDialog::getOpenFileName(this, "Load a PCL file", QDir::currentPath(), "*.pcd *.ply");
        if(!file.isEmpty()) auto val = loadPointCloud(file);
    });
    QAction *saveImage = new QAction(QIcon(":/Resources/Icon/icons8-save-as-48.png"),"");
    connect(saveImage, &QAction::triggered, this, [=]{
        auto file = QFileDialog::getSaveFileName(this, "Save to PCD", QDir::currentPath(), "*.pcd *.ply");
        if(!file.isEmpty()) auto val = savePointCloud(file);
    });
    QAction *zoomIn = new QAction(QIcon(":/Resources/Icon/icons8-zoom-in-48.png"), "");
    connect(zoomIn, &QAction::triggered, this, [=]{
        this->setScale(0.2);
    });
    QAction *zoomOut = new QAction(QIcon(":/Resources/Icon/icons8-zoom-out-48.png"), "");
    connect(zoomOut, &QAction::triggered, this, [=]{
        this->setScale(-0.2);
    });
    QAction *original = new QAction(QIcon(":/Resources/Icon/icons8-refresh-48.png"), "");
    connect(original, &QAction::triggered, this, [=]{
        this->viewer->resetCamera();
        this->refreshView();
    });
    actions << loadImage << saveImage << zoomIn << zoomOut << original;
    toolBar.addActions(actions);


    statusBar.setSizeGripEnabled(false);
#if VTK_MAJOR_VERSION > 8
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    setRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(interactor(), this->renderWindow());
#else
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(this->GetInteractor(), this->GetRenderWindow());
#endif
    viewer->setShowFPS(false);


    currentCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    // viewer->registerPointPickingCallback(&Qylon::GraphicsVTKWidget::callbackPicked, static_cast<void*>(this));
    viewer->registerMouseCallback(&Qylon::GraphicsVTKWidget::callbackMouse, static_cast<void*>(this));
    viewer->registerAreaPickingCallback(&Qylon::GraphicsVTKWidget::callbackAreaPicked, static_cast<void*>(this));
    setOriginal();

    connect(this, &GraphicsVTKWidget::sendCameraPosition, infoWidget, &WidgetInformation::setCameraPosition);
    connect(this, &GraphicsVTKWidget::sendCurrentPoint, infoWidget, &WidgetInformation::setCurrentPoint);
}

void Qylon::GraphicsVTKWidget::setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    QMutexLocker locker(&mutex);
    currentCloud = cloud;

    if(viewer->addPointCloud(currentCloud)) viewer->resetCamera();
    else viewer->updatePointCloud(currentCloud);
    pcl::PointXYZRGB minPoint, maxPoint;
    pcl::getMinMax3D(*currentCloud, minPoint, maxPoint);
    qDebug() << minPoint.x << minPoint.y << minPoint.z << maxPoint.x << maxPoint.y << maxPoint.z;

    refreshView();
}

void Qylon::GraphicsVTKWidget::setScale(double factor)
{
    pcl::visualization::Camera camera;
    viewer->getCameraParameters(camera);

    double pos[3] = { camera.pos[0], camera.pos[1], camera.pos[2] };
    double focal[3] = { camera.focal[0], camera.focal[1], camera.focal[2] };
    double direction[3] = { pos[0] - focal[0], pos[1] - focal[1], pos[2] - focal[2] };
    double length = sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
    direction[0] /= length;
    direction[1] /= length;
    direction[2] /= length;

    pos[0] += direction[0] * -factor * length;
    pos[1] += direction[1] * -factor * length;
    pos[2] += direction[2] * -factor * length;

    viewer->setCameraPosition(pos[0], pos[1], pos[2], focal[0], focal[1], focal[2], camera.view[0], camera.view[1], camera.view[2]);
    viewer->spinOnce();
}

void Qylon::GraphicsVTKWidget::setOriginal()
{
    viewer->setCameraPosition(0, 0, -6000, 0, 0, 5000, 0, -1, 0);
    viewer->resetCamera();
}

void Qylon::GraphicsVTKWidget::setCameraPosition(double p_x, double p_y, double p_z, double f_x, double f_y, double f_z, double v_x, double v_y, double v_z)
{
    viewer->setCameraPosition(p_x, p_y, p_z, f_x, f_y, f_z, v_x, v_y, v_z);
}

bool Qylon::GraphicsVTKWidget::loadPointCloud(QString file)
{
    currentCloud->clear();
    if(file.last(3)=="pcd"){
        if(pcl::io::loadPCDFile(file.toStdString(),*currentCloud) == -1){
            qDebug() << "Couldn't open file" << file;
            return false;
        }else{
            setPointCloud(currentCloud);
            viewer->resetCamera();
            return true;
        }
    }else if(file.last(3)=="ply"){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if(pcl::io::loadPLYFile(file.toStdString(),*currentCloud) == -1){
            qDebug() << "Couldn't open file" << file;
            return false;
        }else{
            setPointCloud(currentCloud);
            viewer->resetCamera();
            return true;
        }
    }
    return false;
}

bool Qylon::GraphicsVTKWidget::savePointCloud(QString file)
{
    if(file.last(3)=="pcd"){
        if(pcl::io::savePCDFileASCII(file.toStdString(),*currentCloud) == -1){
            qDebug() << "Couldn't save file" << file;
            return false;
        }else{
            setPointCloud(currentCloud);
            return true;
        }
    }else if(file.last(3)=="ply"){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if(pcl::io::savePLYFileASCII(file.toStdString(),*currentCloud) == -1){
            qDebug() << "Couldn't save file" << file;
            return false;
        }else{
            setPointCloud(currentCloud);
            return true;
        }
    }
    return false;
}

void Qylon::GraphicsVTKWidget::refreshView(){
#if VTK_MAJOR_VERSION > 8
    renderWindow()->Render();
#else
    update();
#endif
}


void Qylon::GraphicsVTKWidget::callbackMouse(const pcl::visualization::MouseEvent &event, void *arg)
{
    GraphicsVTKWidget *widget = static_cast<GraphicsVTKWidget*>(arg);
    widget->viewer->getRenderWindow()->GetRenderers()->GetDebug();
    switch(event.getType()){
    case pcl::visualization::MouseEvent::MouseMove:{
        int x = event.getX();
        int y = event.getY();
        widget->currentMousePoint->setText("X:"+QString::number(x) + ",Y:" + QString::number(y));

        vtkSmartPointer<vtkPointPicker> picker = vtkSmartPointer<vtkPointPicker>::New();
        picker->SetTolerance(0.003);
        picker->Pick(x, y, 0, widget->viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer());

        double picked[3];
        picker->GetPickPosition(picked);

        int pickedIdx = picker->GetPointId();
        if (pickedIdx == -1 || pickedIdx >= widget->currentCloud->points.size()) {
            // qDebug() << "No valid point picked.";
            return;
        }
        // qDebug() << "Picked position: X:" << picked[0] << "Y:" << picked[1] << "Z:" << picked[2];

        pcl::PointXYZRGB pickedPoint;
        pickedPoint.x = picked[0];
        pickedPoint.y = picked[1];
        pickedPoint.z = picked[2];
        emit widget->sendCurrentPoint(pickedPoint.x, pickedPoint.y, pickedPoint.z);

        pcl::visualization::Camera camera;
        widget->viewer->getCameraParameters(camera);
        double cameraPos[3] = {camera.pos[0], camera.pos[1], camera.pos[2]};

        double distance = sqrt(
            pow(cameraPos[0] - picked[0], 2) +
            pow(cameraPos[1] - picked[1], 2) +
            pow(cameraPos[2] - picked[2], 2)
            );
        double sphereSize = distance * 0.005;

        widget->viewer->removeShape("sphere");
        widget->viewer->addSphere(pickedPoint, sphereSize, 1.0, 0.0, 0.0, "sphere");
        widget->refreshView();
    }break;
    case pcl::visualization::MouseEvent::MouseButtonPress:{}break;
    case pcl::visualization::MouseEvent::MouseButtonRelease:{}break;
    case pcl::visualization::MouseEvent::MouseScrollDown:{}break;
    case pcl::visualization::MouseEvent::MouseScrollUp:{}break;
    case pcl::visualization::MouseEvent::MouseDblClick:{}break;
    default:{}
    }
}

void Qylon::GraphicsVTKWidget::callbackAreaPicked(const pcl::visualization::AreaPickingEvent &event, void *arg)
{
    qDebug() << "Area picked";
    pcl::Indices indices;
    event.getPointsIndices(indices);
    GraphicsVTKWidget *widget = static_cast<GraphicsVTKWidget*>(arg);

}
bool Qylon::GraphicsVTKWidget::event(QEvent *event)
{
    PCLQVTKWidget::event(event);
    if(!viewer) return true;

    pcl::visualization::Camera camera;
    viewer->getCameraParameters(camera);
    double pos[3] = {camera.pos[0], camera.pos[1], camera.pos[2]};
    double focal[3] = {camera.focal[0], camera.focal[1], camera.focal[2]};
    double vector[3] = {camera.view[0], camera.view[1], camera.view[2]};

    emit sendCameraPosition(camera.pos[0], camera.pos[1], camera.pos[2],
                            camera.focal[0], camera.focal[1], camera.focal[2],
                            camera.view[0], camera.view[1], camera.view[2]);
    return true;
}

void Qylon::GraphicsVTKWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->modifiers() & Qt::ShiftModifier) {
        mousePressed = true;
        this->interactor()->Disable();

    } else {
        PCLQVTKWidget::mousePressEvent(event);
    }
}

void Qylon::GraphicsVTKWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (mousePressed && (event->modifiers() & Qt::ShiftModifier)) {
        qDebug() << "here";
    } else {
        PCLQVTKWidget::mouseMoveEvent(event);
    }
}

void Qylon::GraphicsVTKWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (mousePressed && (event->modifiers() & Qt::ShiftModifier)) {
        mousePressed = false;
        this->interactor()->Enable();
    } else {
        PCLQVTKWidget::mouseReleaseEvent(event);
    }
}

/*
void Qylon::GraphicsVTKWidget::mousePressEvent(QMouseEvent *event)
{
    PCLQVTKWidget::mousePressEvent(event);
    if (!currentCloud->empty()) {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*currentCloud, centroid);
        currentPicked[0] = centroid[0];
        currentPicked[1] = centroid[1];
        currentPicked[2] = centroid[2];
    }
    mousePressed = true;
}


void Qylon::GraphicsVTKWidget::mouseMoveEvent(QMouseEvent *event)
{
    PCLQVTKWidget::mouseMoveEvent(event);
    if(mousePressed && !(event->modifiers() & Qt::ShiftModifier || event->modifiers() & Qt::ControlModifier)){
        pcl::visualization::Camera camera;
        viewer->getCameraParameters(camera);
        viewer->setCameraPosition(
            camera.pos[0], camera.pos[1], camera.pos[2],  // Camera position
            currentPicked[0], currentPicked[1], currentPicked[2],  // Focal point
            camera.view[0], camera.view[1], camera.view[2]  // Up vector
            );
    }
}

void Qylon::GraphicsVTKWidget::mouseReleaseEvent(QMouseEvent *event)
{
    PCLQVTKWidget::mouseReleaseEvent(event);
    mousePressed = false;
}


*/
#endif
