#ifndef CAMERA_H
#define CAMERA_H
#ifdef PYLON_ENABLED

#ifdef PCL_ENABLED
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pylon/BlazeInstantCamera.h>
#endif

#include <QObject>
#include <QImage>
#include <QMutexLocker>
#include <QThread>
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
    const void *getBuffer() const;

    bool openCamera(QString cameraName);
    void closeCamera();

    void singleGrab();
    void sequentialGrab(int numFrame);
    void continuousGrab();
    void stopGrab();
    bool isOpened();

    void softwareTriggerReady(bool on);
    QImage softwareTrigger();

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

    bool softwareTriggerMode = false;

    mutable QMutex memberLock;
    mutable QMutex imageLock;
    QImage currentImage;
    void* currentBuffer;
    QImage currentConfidence;
    QImage currentIntensity;

    Pylon::CImageFormatConverter formatConverter;
    Pylon::CAcquireContinuousConfiguration* acquireConfig = nullptr;

#ifdef PCL_ENABLED
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

class CameraThread : public QThread{
    Q_OBJECT
public:
    CameraThread(QObject *parent=nullptr) : QThread(parent){}
    void run() override{
        while(isInterruptionRequested()){
            try{

            }catch(const Pylon::GenericException &e){
                // log(QString::fromStdString(e.GetDescription()));
            }
        }
    }

};
}
inline QImage convertPylonImageToQImage(Pylon::CPylonImage pylonImg){
    Pylon::CImageFormatConverter converter;
    QImage::Format format = QImage::Format_Invalid;
    switch(pylonImg.GetPixelType()){
    case Pylon::PixelType_Undefined:
    case Pylon::PixelType_Mono1packed:
    case Pylon::PixelType_Mono2packed:
    case Pylon::PixelType_Mono4packed:
    case Pylon::PixelType_Mono8:
    case Pylon::PixelType_Mono8signed:
        format = QImage::Format_Grayscale8;
        break;
    case Pylon::PixelType_Mono10:
    case Pylon::PixelType_Mono10packed:
    case Pylon::PixelType_Mono10p:
    case Pylon::PixelType_Mono12:
    case Pylon::PixelType_Mono12packed:
    case Pylon::PixelType_Mono12p:
    case Pylon::PixelType_Mono16:
        format = QImage::Format_Grayscale16;
        break;
    case Pylon::PixelType_BayerGR8:
    case Pylon::PixelType_BayerRG8:
    case Pylon::PixelType_BayerGB8:
    case Pylon::PixelType_BayerBG8:
    case Pylon::PixelType_BayerGR10:
    case Pylon::PixelType_BayerRG10:
    case Pylon::PixelType_BayerGB10:
    case Pylon::PixelType_BayerBG10:
    case Pylon::PixelType_BayerGR12:
    case Pylon::PixelType_BayerRG12:
    case Pylon::PixelType_BayerGB12:
    case Pylon::PixelType_BayerBG12:
    case Pylon::PixelType_RGB8packed:
    case Pylon::PixelType_BGR8packed:
    case Pylon::PixelType_RGBA8packed:
    case Pylon::PixelType_BGRA8packed:
    case Pylon::PixelType_RGB10packed:
    case Pylon::PixelType_BGR10packed:
    case Pylon::PixelType_RGB12packed:
    case Pylon::PixelType_BGR12packed:
    case Pylon::PixelType_RGB16packed:
    case Pylon::PixelType_BGR10V1packed:
    case Pylon::PixelType_BGR10V2packed:
    case Pylon::PixelType_YUV411packed:
    case Pylon::PixelType_YUV422packed:
    case Pylon::PixelType_YUV444packed:
    case Pylon::PixelType_RGB8planar:
    case Pylon::PixelType_RGB10planar:
    case Pylon::PixelType_RGB12planar:
    case Pylon::PixelType_RGB16planar:
    case Pylon::PixelType_YUV422_YUYV_Packed:
    case Pylon::PixelType_YUV444planar:
    case Pylon::PixelType_YUV422planar:
    case Pylon::PixelType_YUV420planar:
    case Pylon::PixelType_YCbCr420_8_YY_CbCr_Semiplanar:
    case Pylon::PixelType_YCbCr422_8_YY_CbCr_Semiplanar:
    case Pylon::PixelType_BayerGR12Packed:
    case Pylon::PixelType_BayerRG12Packed:
    case Pylon::PixelType_BayerGB12Packed:
    case Pylon::PixelType_BayerBG12Packed:
    case Pylon::PixelType_BayerGR10p:
    case Pylon::PixelType_BayerRG10p:
    case Pylon::PixelType_BayerGB10p:
    case Pylon::PixelType_BayerBG10p:
    case Pylon::PixelType_BayerGR12p:
    case Pylon::PixelType_BayerRG12p:
    case Pylon::PixelType_BayerGB12p:
    case Pylon::PixelType_BayerBG12p:
    case Pylon::PixelType_BayerGR16:
    case Pylon::PixelType_BayerRG16:
    case Pylon::PixelType_BayerGB16:
    case Pylon::PixelType_BayerBG16:
    case Pylon::PixelType_RGB12V1packed:
        format = QImage::Format_RGB32;
        break;
    case Pylon::PixelType_Double:
    case Pylon::PixelType_Confidence8:
    case Pylon::PixelType_Confidence16:
    case Pylon::PixelType_Coord3D_C8:
    case Pylon::PixelType_Coord3D_C16:
    case Pylon::PixelType_Coord3D_ABC32f:
    case Pylon::PixelType_Data8:
    case Pylon::PixelType_Data8s:
    case Pylon::PixelType_Data16:
    case Pylon::PixelType_Data16s:
    case Pylon::PixelType_Data32:
    case Pylon::PixelType_Data32s:
    case Pylon::PixelType_Data64:
    case Pylon::PixelType_Data64s:
    case Pylon::PixelType_Data32f:
    case Pylon::PixelType_Data64f:
    case Pylon::PixelType_Error8:
        break;
    }
    QImage outImage(pylonImg.GetWidth(), pylonImg.GetHeight(), format);
    if(format != QImage::Format_RGB32){
        int width = pylonImg.GetWidth();
        int height = pylonImg.GetHeight();
        const uchar* buffer = static_cast<const uchar*>(pylonImg.GetBuffer());
        outImage = QImage(buffer, width, height, width, format).copy();
    }else{
        converter.OutputPixelFormat = Pylon::PixelType_BGRA8packed;
        converter.Convert(outImage.bits(), outImage.sizeInBytes(), pylonImg);
    }
    return outImage;
}
#endif
#endif // CAMERA_H
