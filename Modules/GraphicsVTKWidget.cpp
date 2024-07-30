#ifdef PCL_ENABLED
#include "GraphicsVTKWidget.h"
Qylon::GraphicsVTKWidget::GraphicsVTKWidget(QWidget *parent){
#if VTK_MAJOR_VERSION > 8
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    setRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(this->interactor(), this->renderWindow());
#else
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(this->GetInteractor(), this->GetRenderWindow());
#endif
    viewer->setBackgroundColor(0.1,0.1,0.1);
    viewer->resetCamera();
    viewer->setCameraPosition(0, 0, -4000, 0, 0, 3000, 0, -1, 0);
}

void Qylon::GraphicsVTKWidget::setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    QMutexLocker locker(&mutex);
    if(!isAdded){
        viewer->addPointCloud(cloud, "cloud");
        isAdded = true;
    }else{
        viewer->updatePointCloud(cloud, "cloud");
    }
    refreshView();
}

void Qylon::GraphicsVTKWidget::refreshView(){
#if VTK_MAJOR_VERSION > 8
    this->renderWindow()->Render();
#else
    this->update();
#endif
}
#endif
