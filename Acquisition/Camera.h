#ifndef CAMERA_H
#define CAMERA_H

#ifdef PCL_ENABLED
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#endif
#ifdef OPENCV_ENABLED
#include <opencv2/highgui.hpp>
#include <pylon/BlazeInstantCamera.h>
#endif

#include <QObject>
#include <QImage>
#include <QMutexLocker>
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

namespace Qylon{
class Qylon;
class CameraWidget;
class Camera : public QObject,
        public Pylon::CImageEventHandler,
        public Pylon::CConfigurationEventHandler,
        public Pylon::CCameraEventHandler
{
    Q_OBJECT
public:
    Camera(Qylon *parentQylon=nullptr);
    virtual ~Camera();

    const QImage &getImage() const;

    bool openCamera(QString cameraName);
    void closeCamera();

    void singleGrab();
    void continuousGrab();
    void stopGrab();
    bool isOpened();

    Qylon *getQylon();
    Pylon::CBaslerUniversalInstantCamera *getInstantCamera();

    QWidget *getWidget();

    GenApi::INode *getNodemap(GenICam::gcstring nodeName);

    GenApi_3_1_Basler_pylon::CIntegerPtr setNodeValue(QString node, int &value);
    void setNode(QString node, float &value);
    void setNode(QString node, bool &value);
    void setNode(QString node, QString &value);


    QMutex* drawLock() const;

#ifdef PCL_ENABLED
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertGrabResultToPointCloud(const Pylon::CGrabResultPtr& grabResult);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloudData();
    const QImage &getConfidence() const;
    const QImage &getIntensity() const;
#endif

signals:
    void grabbed();
    void grabbedPointCloud();
    void grabbedConfidence();
    void grabbedIntensity();
    void grabbingState(bool isGrabbing);
    void removed();
    void connected();

private:
    Qylon *parent = nullptr;
    Pylon::CBaslerUniversalInstantCamera currentInstantCamera;

    mutable QMutex memberLock;
    mutable QMutex imageLock;
    QImage currentImage;
    QImage currentConfidence;
    QImage currentIntensity;

    Pylon::CImageFormatConverter formatConverter;

#ifdef PCL_ENABLED
    std::shared_ptr<pcl::visualization::CloudViewer> viewer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcPtr;
#endif

    CameraWidget *widget;

protected:

    // Pylon::CImageEventHandler functions
    virtual void OnImagesSkipped( Pylon::CInstantCamera& camera, size_t countOfSkippedImages );
    virtual void OnImageGrabbed( Pylon::CInstantCamera& camera, const Pylon::CGrabResultPtr& grabResult );

    // Pylon::CConfigurationEventHandler functions
    //    virtual void OnAttach( Pylon::CInstantCamera& camera );
    virtual void OnAttached( Pylon::CInstantCamera& camera );
    //    virtual void OnDetach( Pylon::CInstantCamera& camera );
    virtual void OnDetached( Pylon::CInstantCamera& camera );
    virtual void OnDestroy( Pylon::CInstantCamera& camera );
    //    virtual void OnDestroyed( Pylon::CInstantCamera& camera );
    //    virtual void OnOpen( Pylon::CInstantCamera& camera );
    virtual void OnOpened( Pylon::CInstantCamera& camera );
    //    virtual void OnClose( Pylon::CInstantCamera& camera );
    virtual void OnClosed( Pylon::CInstantCamera& camera );
    //    virtual void OnGrabStart( Pylon::CInstantCamera& camera );
    virtual void OnGrabStarted( Pylon::CInstantCamera& camera );
    //    virtual void OnGrabStop( Pylon::CInstantCamera& camera );
    virtual void OnGrabStopped( Pylon::CInstantCamera& camera );
    virtual void OnGrabError( Pylon::CInstantCamera& camera, const char* errorMessage );
    virtual void OnCameraDeviceRemoved( Pylon::CInstantCamera& camera );

    // Pylon::CCameraEventHandler function
    virtual void OnCameraEvent( Pylon::CInstantCamera& camera, intptr_t userProvidedId, GenApi::INode* pNode );
};
}

#endif // CAMERA_H
