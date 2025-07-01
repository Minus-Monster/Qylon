#ifdef PYLON_ENABLED
#include "Qylon.h"
#include "Camera.h"
#include "CameraWidget.h"

#ifdef PCL_ENABLED
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <omp.h>
#endif

Qylon::Camera::Camera(Qylon *parentQylon) : parent(parentQylon){
    widget = new CameraWidget(this);
    connect(this, &Camera::grabbingState, widget, &CameraWidget::grabbingState);
    connect(this, &Camera::connectionStatus, widget, &CameraWidget::connectionStatus);

    acquireConfig = new Pylon::CAcquireContinuousConfiguration;
    // The order of these codes is critically importatnt
    currentInstantCamera.RegisterImageEventHandler(this, Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_None);
    currentInstantCamera.RegisterConfiguration(acquireConfig, Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_Delete );
    currentInstantCamera.RegisterConfiguration(this, Pylon::RegistrationMode_Append, Pylon::Cleanup_None );
}

Qylon::Camera::~Camera(){
    deleteLater();
    delete widget;
}

/// This function could be used when the widget recieved the signal 'grabbed()'
/// If you got the signal, you can call this function to get the result image.
const QImage &Qylon::Camera::getImage() const
{
    return currentImage;
}

const void *Qylon::Camera::getBuffer() const
{
    return currentBuffer;
}


/// Try to open a camera by the camera's friendly name.
/// It will return True or False depending on the result.
bool Qylon::Camera::openCamera(QString cameraName){
    QMutexLocker lock(&memberLock);

    try{
        Pylon::IPylonDevice* pDevice = Pylon::CTlFactory::GetInstance().CreateDevice(parent->getCameraIndexfromName(cameraName));
        currentInstantCamera.Attach(pDevice, Pylon::Cleanup_Delete);
        currentInstantCamera.Open();

        if(QString(currentInstantCamera.GetDeviceInfo().GetModelName()).contains("blaze")){
            //Enable streaming in GenDC format
            auto &nodemap = currentInstantCamera.GetNodeMap();
            auto genDCStreamingMode = Pylon::CEnumParameter(nodemap, "GenDCStreamingMode");
            if (genDCStreamingMode.IsWritable()){
                genDCStreamingMode.SetValue("On");
            }
            auto componentSelector = Pylon::CEnumParameter(nodemap, "ComponentSelector");
            auto componentEnable = Pylon::CBooleanParameter(nodemap, "ComponentEnable");
            auto pixelFormat = Pylon::CEnumParameter(nodemap, "PixelFormat");
            if (componentSelector.IsWritable()){
                std::pair<Pylon::String_t, Pylon::String_t> formats[]{
                                                                      {"Range", "Coord3D_ABC32f"},
                                                                      {"Intensity", "Mono16"},
                                                                      {"Confidence", "Confidence16"} };
                for (const auto& format : formats){
                    componentSelector.SetValue(format.first);
                    if (componentEnable.IsWritable()){
                        componentEnable.SetValue(true);
                    }
                    if (pixelFormat.IsWritable()){
                        pixelFormat.SetValue(format.second);
                    }
                }
            }
            for (auto axis : {"CoordinateA", "CoordinateB", "CoordinateC"}){
                currentInstantCamera.Scan3dCoordinateSelector.SetValue(axis);
                currentInstantCamera.Scan3dInvalidDataValue.SetValue(std::numeric_limits<float>::quiet_NaN());
            }
        }
        return true;
    }catch (const Pylon::GenericException &e){
        Qylon::log(e.GetDescription());
    }catch (QString error){
        Qylon::log(error);
    }
    lock.unlock();
    closeCamera();
    return false;
}

/// This function hasn't any returning data.
/// When it called, All of the camera's features will be ended.
void Qylon::Camera::closeCamera(){
    QMutexLocker lock(&memberLock);
    try{
        if(!this->isOpened()) throw QString("Camera is not opened.");
        stopGrab();

        QImage dummy;
        currentImage.swap(dummy);
        currentInstantCamera.DestroyDevice();
    }
    catch(const Pylon::GenericException &e){ Qylon::log(QString::fromStdString(e.GetDescription()));}
    catch(QString e){ Qylon::log(e);}

}

void Qylon::Camera::singleGrab()
{
    try{
        if(!this->isOpened()) throw QString("Camera is not opened.");
        currentInstantCamera.AcquisitionMode.TrySetValue(Basler_UniversalCameraParams::AcquisitionMode_SingleFrame);
        currentInstantCamera.StartGrabbing(1, Pylon::GrabStrategy_OneByOne, Pylon::GrabLoop_ProvidedByInstantCamera);
    }catch(const QString error){ parent->log("Single Grabbing Failed. " + error);}
    catch(const Pylon::GenericException &e){ parent->log(("Single Grabbing Failed. " + QString::fromStdString(e.GetDescription())));}
}

void Qylon::Camera::sequentialGrab(int numFrame)
{
    try{
        if(!this->isOpened()) throw QString("Camera is not opened.");
        currentInstantCamera.AcquisitionMode.TrySetValue(Basler_UniversalCameraParams::AcquisitionMode_MultiFrame);
        currentInstantCamera.StartGrabbing(numFrame, Pylon::GrabStrategy_OneByOne, Pylon::GrabLoop_ProvidedByInstantCamera);
    }catch(const QString error){ Qylon::log("Sequential Grabbing Failed. " + error);}
    catch(const Pylon::GenericException &e){ Qylon::log(("Sequential Grabbing Failed. " + QString::fromStdString(e.GetDescription())));}
}

void Qylon::Camera::continuousGrab()
{
    try{
        if(!this->isOpened()) throw QString("Camera is not opened.");
        currentInstantCamera.AcquisitionMode.TrySetValue(Basler_UniversalCameraParams::AcquisitionMode_Continuous);
        currentInstantCamera.StartGrabbing(Pylon::GrabStrategy_OneByOne, Pylon::GrabLoop_ProvidedByInstantCamera);
    }catch(const QString error){ Qylon::log("Continuous Grabbing Failed. " + error);}
    catch(const Pylon::GenericException &e){ Qylon::log(("Continuous Grabbing Failed. " + QString::fromStdString(e.GetDescription())));}
}

void Qylon::Camera::stopGrab()
{
    try{
        if(!this->isOpened()) throw QString("Camera is not opened.");
        currentInstantCamera.StopGrabbing();
    }catch(const Pylon::GenericException &e){ Qylon::log(QString::fromStdString(e.GetDescription()));}
}

bool Qylon::Camera::isOpened(){
    return currentInstantCamera.IsOpen();
}

// Using it after opening the camera
void Qylon::Camera::softwareTriggerReady(bool on)
{
    if(on){
        currentInstantCamera.DeregisterImageEventHandler(this);
        currentInstantCamera.DeregisterConfiguration(acquireConfig);
        currentInstantCamera.DeregisterConfiguration(this);
        acquireConfig=nullptr;

        currentInstantCamera.TriggerMode.SetValue(Basler_UniversalCameraParams::TriggerMode_On);
        currentInstantCamera.TriggerSource.SetValue(Basler_UniversalCameraParams::TriggerSource_Software);

        currentInstantCamera.StartGrabbing();
        emit widget->nodeUpdated();
        emit grabbingState(true);
    }else{
        currentInstantCamera.StopGrabbing();

        // The order of these codes is critically importatnt
        currentInstantCamera.TriggerMode.SetValue(Basler_UniversalCameraParams::TriggerMode_Off);

        acquireConfig = new Pylon::CAcquireContinuousConfiguration;
        currentInstantCamera.RegisterImageEventHandler(this, Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_None);
        currentInstantCamera.RegisterConfiguration(acquireConfig, Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_Delete );
        currentInstantCamera.RegisterConfiguration(this, Pylon::RegistrationMode_Append, Pylon::Cleanup_None );
        emit widget->nodeUpdated();
        emit grabbingState(false);
    }
}

QImage Qylon::Camera::softwareTrigger()
{
    try{
        currentInstantCamera.ExecuteSoftwareTrigger();
        Pylon::CGrabResultPtr ptrGrabResult;
        currentInstantCamera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        if (ptrGrabResult->GrabSucceeded()){
            Pylon::CPylonImage image;
            image.AttachGrabResultBuffer(ptrGrabResult);
            return convertPylonImageToQImage(image);
        }
    }catch(const Pylon::GenericException &e){
        Qylon::log("Software trigger acquisition is failed.");
    }
    return QImage();
}

Pylon::CBaslerUniversalInstantCamera *Qylon::Camera::getInstantCamera(){
    return &currentInstantCamera;
}

QWidget *Qylon::Camera::getWidget(){
    return widget;
}

GenApi::INode *Qylon::Camera::getNode(QString nodeName)
{
    /// Packet size : GevSCPSPacketSize , Interpacketdelay = GevSCPD
    try {
        return currentInstantCamera.GetNodeMap().GetNode(nodeName.toStdString().c_str());
    } catch (const Pylon::GenericException &e) {
        Qylon::log(nodeName + ": " + e.GetDescription());
    }
    return nullptr;

}

void Qylon::Camera::OnImagesSkipped(Pylon::CInstantCamera &camera, size_t countOfSkippedImages)
{
    parent->log(QString(camera.GetDeviceInfo().GetFriendlyName().c_str()) + " image skipped " + QString::number(countOfSkippedImages));
    PYLON_UNUSED( camera );
}

void Qylon::Camera::OnImageGrabbed(Pylon::CInstantCamera &camera, const Pylon::CGrabResultPtr &grabResult)
{
    // Lock to ensure thread safety while working with shared resources
    QMutexLocker lockGrab(&memberLock);
    QMutexLocker lockImage(&imageLock);
    if (!(grabResult.IsValid() && grabResult->GrabSucceeded())) return;

    // Only process images if the camera is "blaze"
    if (QString(camera.GetDeviceInfo().GetModelName().c_str()).contains("blaze")) {
#ifdef PCL_ENABLED
        try {
            auto container = grabResult->GetDataContainer();

            // Use OpenMP to process components in parallel
#pragma omp parallel for
            for (size_t i = 0; i < container.GetDataComponentCount(); ++i) {
                auto component = container.GetDataComponent(i);
                if (component.IsValid()) {
                    switch (component.GetComponentType()) {
                    case Pylon::ComponentType_Intensity:
                        try {
                            currentIntensity = QImage(component.GetWidth(), component.GetHeight(), QImage::Format_Grayscale16);
                            memcpy(currentIntensity.bits(), component.GetData(), component.GetDataSize());
                            emit grabbedIntensity();
                        } catch (const Pylon::GenericException &e) {
                            Qylon::log(e.GetDescription());
                        }
                        break;

                    case Pylon::ComponentType_Range:   // Point Cloud Data
                        pcPtr = convertGrabResultToPointCloud(grabResult);
                        emit grabbedPointCloud();
                        break;

                    case Pylon::ComponentType_Confidence:
                        currentConfidence = QImage(component.GetWidth(), component.GetHeight(), QImage::Format_Grayscale16);
                        memcpy(currentConfidence.bits(), component.GetData(), component.GetDataSize());
                        emit grabbedConfidence();
                        break;

                    default:
                        break;  // Skip undefined or unsupported components
                    }
                }
            }
            emit grabbed();
        } catch (const Pylon::GenericException &e) {
            Qylon::log("[ERROR WHILE MAKING GRABBED IMAGE]");
        }
#endif
    } else {  // Handle non-blaze cameras
        try {
            Pylon::CPylonImage image;
            image.AttachGrabResultBuffer(grabResult);
            currentImage = convertPylonImageToQImage(image);
            emit grabbed();
        } catch (const Pylon::GenericException &e) {
            Qylon::log(QString("Image conversion error occurred.") + e.what());
        }
    }
    lockGrab.unlock();
    lockImage.unlock();  // Unlock the mutex after the image processing
    PYLON_UNUSED(camera);
}


void Qylon::Camera::OnAttached(Pylon::CInstantCamera &camera)
{
    parent->log(QString(camera.GetDeviceInfo().GetFriendlyName().c_str()) + " is attached.");
    PYLON_UNUSED( camera );
}
void Qylon::Camera::OnDetached(Pylon::CInstantCamera &camera)
{
    parent->log(QString(camera.GetDeviceInfo().GetFriendlyName().c_str()) + " is detached.");
    PYLON_UNUSED( camera );
}
void Qylon::Camera::OnDestroy(Pylon::CInstantCamera &camera)
{
    parent->log(QString(camera.GetDeviceInfo().GetFriendlyName().c_str()) + " will be destroyed.");
    PYLON_UNUSED( camera );
}
void Qylon::Camera::OnOpened(Pylon::CInstantCamera &camera)
{
    parent->log(QString(camera.GetDeviceInfo().GetFriendlyName().c_str()) + " is opened.");
    emit connectionStatus(true);
    PYLON_UNUSED( camera );
}
void Qylon::Camera::OnClosed(Pylon::CInstantCamera &camera)
{
    emit connectionStatus(false);
    parent->log(QString(camera.GetDeviceInfo().GetFriendlyName().c_str()) + " is closed.");
    PYLON_UNUSED( camera );
}
void Qylon::Camera::OnGrabStarted(Pylon::CInstantCamera &camera)
{
    parent->log(QString(camera.GetDeviceInfo().GetFriendlyName().c_str()) + " is starting to grab.");
    emit grabbingState(true);
    PYLON_UNUSED( camera );
}
void Qylon::Camera::OnGrabStopped(Pylon::CInstantCamera &camera)
{
    parent->log(QString(camera.GetDeviceInfo().GetFriendlyName().c_str()) + " is stopped grabbing.");
    emit grabbingState(false);
    PYLON_UNUSED( camera );
}
void Qylon::Camera::OnGrabError(Pylon::CInstantCamera &camera, const char *errorMessage)
{
    this->parent->log(errorMessage);
    PYLON_UNUSED( camera );
}
void Qylon::Camera::OnCameraDeviceRemoved(Pylon::CInstantCamera &camera)
{
    parent->log(QString(camera.GetDeviceInfo().GetFriendlyName().c_str()) + " is removed.");
    emit connectionStatus(false);

    PYLON_UNUSED( camera );
}
void Qylon::Camera::OnCameraEvent(Pylon::CInstantCamera &camera, intptr_t userProvidedId, GenApi::INode *pNode)
{
    qDebug() << "The event is occured from" << camera.GetDeviceInfo().GetFriendlyName()  << pNode->GetDisplayName() << pNode->GetNodeMap()->GetNode("TriggerMode");
}

QMutex *Qylon::Camera::drawLock() const
{
    return &imageLock;
}

Qylon::Qylon *Qylon::Camera::getQylon(){
    return parent;
}

#ifdef PCL_ENABLED
struct Point{
    float x;
    float y;
    float z;
};

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Qylon::Camera::convertGrabResultToPointCloud(const Pylon::CGrabResultPtr& grabResult)
{
    const auto container = grabResult->GetDataContainer();
    const auto rangeComponent = container.GetDataComponent(0);
    const auto intensityComponent = container.GetDataComponent(1);

    const uint32_t width = rangeComponent.GetWidth();
    const uint32_t height = rangeComponent.GetHeight();

    auto cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(width * height);

    const auto* pSrcPoint = reinterpret_cast<const Point*>(rangeComponent.GetData());
    const auto* pIntensity = reinterpret_cast<const uint16_t*>(intensityComponent.GetData());

#pragma omp parallel for
    for (uint32_t i = 0; i < width * height; ++i) {
        auto& pt = cloud->points[i];
        const auto& src = pSrcPoint[i];

        pt.x = src.x;
        pt.y = src.y;
        pt.z = src.z;

        pt.r = pt.g = pt.b = (uint8_t)(pIntensity[i] >>8);
    }

    return cloud;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr Qylon::Camera::getPointCloudData()
{
    return pcPtr;
}

const QImage &Qylon::Camera::getConfidence() const
{
    return currentConfidence;
}

const QImage &Qylon::Camera::getIntensity() const
{
    return currentIntensity;
}
#endif
#endif
