#ifdef PCL_ENABLED
#include "GraphicsVTKWidget.h"

Qylon::GraphicsVTKWidget::GraphicsVTKWidget(){

    defaultCamera.pos[0] = 0; defaultCamera.pos[1] = 0; defaultCamera.pos[2] = -500;
    defaultCamera.focal[0] = 0; defaultCamera.focal[1] = 0; defaultCamera.focal[2] = 0;
    defaultCamera.view[0] = 0; defaultCamera.view[1] = -1; defaultCamera.view[2] = 0;



    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    this->setRenderWindow(viewer->getRenderWindow());
    this->setAttribute(Qt::WA_NativeWindow);
    viewer->setupInteractor(this->interactor(), this->renderWindow());
    viewer->removeAllCoordinateSystems();
    viewer->setShowFPS(false);

    //        viewer->registerMouseCallback(mouseCallBack, (void*)&viewer);
    //        viewer->registerKeyboardCallback(keyboardCallBack, (void*)&viewer);
    //        viewer->registerPointPickingCallback(pickingPointCallBack);
    viewer->setCameraParameters(defaultCamera);




}

void Qylon::GraphicsVTKWidget::setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudData, QString pointCloudName){
    if(!viewer->updatePointCloud(cloudData, pointCloudName.toStdString())){
        viewer->addPointCloud(cloudData, pointCloudName.toStdString());
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, pointCloudName.toStdString());
    }
    pcl::visualization::Camera camera;
    viewer->getCameraParameters(camera);
    QString data = QString("Pos[" + QString::number(camera.pos[0]) +","+ QString::number(camera.pos[1])+"," + QString::number(camera.pos[2]) +"] / " +
                           "Focal[" + QString::number(camera.focal[0]) +","+ QString::number(camera.focal[1]) +","+ QString::number(camera.focal[2]) +"] /"+
                           "View[" + QString::number(camera.view[0]) +","+ QString::number(camera.view[1]) +","+ QString::number(camera.view[2]) + "]");
    qDebug() << "Update Camera position" <<  data;

    viewer->getRenderWindow()->Render();
}

void Qylon::GraphicsVTKWidget::setViewUp(){
    pcl::visualization::Camera camera;
    viewer->getCameraParameters(camera);
    double currentVal = -200.;
    viewer->setCameraPosition(camera.pos[0] , camera.pos[1]+ currentVal, camera.pos[2],
                              camera.focal[0], camera.focal[1] , camera.focal[2],
                              camera.view[0], camera.view[1], camera.view[2]);
    updateCameraPosition();
}

void Qylon::GraphicsVTKWidget::setViewDown(){
    pcl::visualization::Camera camera;
    viewer->getCameraParameters(camera);
    double currentVal = 200.;
    viewer->setCameraPosition(camera.pos[0] , camera.pos[1]+ currentVal, camera.pos[2],
                              camera.focal[0], camera.focal[1] , camera.focal[2],
                              camera.view[0], camera.view[1], camera.view[2]);
    updateCameraPosition();
}

void Qylon::GraphicsVTKWidget::setViewLeft(){
    pcl::visualization::Camera camera;
    viewer->getCameraParameters(camera);
    double currentVal = -1000.;
    viewer->setCameraPosition(camera.pos[0] + currentVal, camera.pos[1], camera.pos[2],
                              camera.focal[0], camera.focal[1], camera.focal[2],
                              camera.view[0], camera.view[1], camera.view[2]);
    updateCameraPosition();
}

void Qylon::GraphicsVTKWidget::setViewRight(){
    pcl::visualization::Camera camera;
    viewer->getCameraParameters(camera);
    double currentVal = 1000.;
    viewer->setCameraPosition(camera.pos[0] + currentVal, camera.pos[1], camera.pos[2],
                              camera.focal[0], camera.focal[1], camera.focal[2],
                              camera.view[0], camera.view[1], camera.view[2]);
    // view ranges from -1 to 1.
    updateCameraPosition();
}

void Qylon::GraphicsVTKWidget::setScale(float scale){
    pcl::visualization::Camera camera;
    viewer->getCameraParameters(camera);
    double currentVal = 0.;
    if(scale > 1.){ // Reverse to be thank as similar
        scale = 0.8;
    }else{
        scale = 1.2;
    }
    viewer->setCameraPosition(camera.pos[0], camera.pos[1], camera.pos[2] * scale,
                              camera.focal[0], camera.focal[1], camera.focal[2],
                              camera.view[0], camera.view[1], camera.view[2]);
    updateCameraPosition();
}

void Qylon::GraphicsVTKWidget::resetScale(){
    viewer->setCameraParameters(defaultCamera);
    updateCameraPosition();
}

void Qylon::GraphicsVTKWidget::updateCameraPosition(){

    pcl::visualization::Camera camera;
    viewer->getCameraParameters(camera);
    QString data = QString("Pos[" + QString::number(camera.pos[0]) +","+ QString::number(camera.pos[1])+"," + QString::number(camera.pos[2]) +"] / " +
                           "Focal[" + QString::number(camera.focal[0]) +","+ QString::number(camera.focal[1]) +","+ QString::number(camera.focal[2]) +"] /"+
                           "View[" + QString::number(camera.view[0]) +","+ QString::number(camera.view[1]) +","+ QString::number(camera.view[2]) + "]");
    qDebug() << "Update Camera position" <<  data;
    viewer->getRenderWindow()->Render();
    emit currentCameraPosition(data);

}

void Qylon::GraphicsVTKWidget::pickingPointCallBack(const pcl::visualization::PointPickingEvent &event){
    float x,y,z;
    event.getPoint(x,y,z);
    qDebug()<< "Picking" << x << y << z;
}

void Qylon::GraphicsVTKWidget::mouseCallBack(const pcl::visualization::MouseEvent &event, void *viewer_void){
    // To remove the mouse event
    if(event.getType() == pcl::visualization::MouseEvent::MouseScrollUp){}
    if(event.getType() == pcl::visualization::MouseEvent::MouseScrollDown){}
    if(event.getType() == pcl::visualization::MouseEvent::MouseButtonPress){}
    if(event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease){}
    if(event.getType() == pcl::visualization::MouseEvent::MouseMove){}
}
#endif
