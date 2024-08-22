#ifdef PCL_ENABLED
#include "GraphicsVTKWidget.h"

Qylon::GraphicsVTKWidget::GraphicsVTKWidget(QWidget *parent){
    setLayout(&layout);
    layout.addWidget(&toolBar);
    layout.addWidget(&statusBar,0, Qt::AlignBottom);

    currentCameraPosition = new QLabel;
    statusBar.addWidget(currentCameraPosition);

    currentPickedPoint = new QLabel;
    statusBar.addWidget(currentPickedPoint);

    QList<QAction*> actions;
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
        this->setOriginal();
    });
    actions << zoomIn << zoomOut << original;
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
    viewer->resetCamera();
    viewer->setCameraPosition(0, 0, 0, 0, 0, 50, 0, -1, 0);
    currentCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    viewer->addPointCloud(currentCloud, "cloud");
    viewer->registerPointPickingCallback(&Qylon::GraphicsVTKWidget::callbackPicked, static_cast<void*>(this));
    refreshView();
}

void Qylon::GraphicsVTKWidget::setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    QMutexLocker locker(&mutex);
    currentCloud = cloud;
    viewer->updatePointCloud(currentCloud, "cloud");
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
    viewer->setCameraPosition(0, 0, 0, 0, 0, 50., 0, -1., 0);
}

void Qylon::GraphicsVTKWidget::refreshView(){
#if VTK_MAJOR_VERSION > 8
    renderWindow()->Render();
#else
    update();
#endif
}

void Qylon::GraphicsVTKWidget::callbackPicked(const pcl::visualization::PointPickingEvent &event, void *arg)
{
    int idx = event.getPointIndex();
    if (idx == -1) {
        qDebug() << "No point picked";
        return;
    }

    float x, y, z;
    event.getPoint(x, y, z);

    GraphicsVTKWidget *widget = static_cast<GraphicsVTKWidget*>(arg);
    widget->currentPickedPoint->setText("[x:" + QString::number(x) +", y:" + QString::number(y) + ", z:" + QString::number(z)+"]");
    widget->currentPickedPoint->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

    // Clicked point in 3D coordinates
    pcl::PointXYZ picked_point(x, y, z);

    // Get camera parameters
    pcl::visualization::Camera camera;
    widget->viewer->getCameraParameters(camera);

    widget->viewer->removeShape("arrow");
    widget->viewer->addSphere(picked_point, 2, 1.0, 0.0, 0.0, "arrow");
    widget->refreshView();
}

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

bool Qylon::GraphicsVTKWidget::event(QEvent *event)
{
    PCLQVTKWidget::event(event);
    if(!viewer) return true;

    pcl::visualization::Camera camera;
    viewer->getCameraParameters(camera);
    double pos[3] = {camera.pos[0], camera.pos[1], camera.pos[2]};
    double focal[3] = {camera.focal[0], camera.focal[1], camera.focal[2]};
    double vector[3] = {camera.view[0], camera.view[1], camera.view[2]};


    currentCameraPosition->setText("Pos["+QString::number(camera.pos[0]) + ", " + QString::number(camera.pos[1]) + ", " + QString::number(camera.pos[2]) + "]\n"
                                   + "Focal["+QString::number(camera.focal[0]) + ", " + QString::number(camera.focal[1]) + ", " + QString::number(camera.focal[2]) + "]\n"
                                   + "View[" +QString::number(camera.view[0]) + ", " + QString::number(camera.view[1]) + ", " + QString::number(camera.view[2]) + "]\n"
                                   );
    currentCameraPosition->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    return true;
}
#endif
