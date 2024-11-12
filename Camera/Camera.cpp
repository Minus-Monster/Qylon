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
    connect(this, &Camera::grabbingState, widget, [=](bool grabbing){
        // should have sending a message that grabbing is started
        emit widget->grabbingState(grabbing);

    });
    connect(this, &Camera::removed, widget, &CameraWidget::disconnectCamera);
    connect(this, &Camera::connected, widget, &CameraWidget::connectedCameraFromOutside);

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
            //Enable all image components, send range data as point cloud.
            //            /*
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
        qDebug() << "Open Setup error" << e.what();
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
    currentInstantCamera.ExecuteSoftwareTrigger();
    Pylon::CGrabResultPtr ptrGrabResult;
    currentInstantCamera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
    if (ptrGrabResult->GrabSucceeded()){
        Pylon::CPylonImage image;
        image.AttachGrabResultBuffer(ptrGrabResult);
        return convertPylonImageToQImage(image);
    }
    Qylon::log("Software trigger acquisition is failed.");
    return QImage();
}

Pylon::CBaslerUniversalInstantCamera *Qylon::Camera::getInstantCamera(){
    return &currentInstantCamera;
}

QWidget *Qylon::Camera::getWidget(){
    return widget;
}

GenApi_3_1_Basler_pylon::INode *Qylon::Camera::getNodemap(GenICam_3_1_Basler_pylon::gcstring nodeName)
{
    /// Packet size : GevSCPSPacketSize , Interpacketdelay = GevSCPD
    try {
        return currentInstantCamera.GetNodeMap().GetNode(nodeName);
    } catch (const Pylon::GenericException &e) {
        getQylon()->log(e.GetDescription());
    }
    return nullptr;

}

GenApi_3_1_Basler_pylon::CIntegerPtr Qylon::Camera::setNodeValue(QString node, int &value)
{
    try{
        GenApi_3_1_Basler_pylon::CIntegerPtr ptr = getNodemap(node.toStdString().c_str());
        ptr->SetValue(value);
        return ptr;
    }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
        parent->log(e.GetDescription());
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
    QMutexLocker lockGrab(&memberLock);
    lockGrab.unlock();

    QMutexLocker lockImage(&imageLock);
    if(!(grabResult.IsValid() && grabResult->GrabSucceeded())) return;

    /// If you need anything of image processing from this buffer
    /// you should implement codes here

    if(QString(camera.GetDeviceInfo().GetModelName().c_str()).contains("blaze")){
        // If this buffer is of blaze camera, The buffer image above code lines is a depth image.
#ifdef PCL_ENABLED
        try{
            auto container = grabResult->GetDataContainer();
            for(size_t i = 0; i < container.GetDataComponentCount(); ++i){
                auto component = container.GetDataComponent(i);
                if(component.IsValid()){
                    switch (component.GetComponentType()){
                    case Pylon::ComponentType_Intensity:{
                        try{
                            currentIntensity = QImage(component.GetWidth(), component.GetHeight(),QImage::Format_Grayscale16);
                            memcpy(currentIntensity.bits(), component.GetData(), component.GetDataSize());

                            emit grabbedIntensity();
                        }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
                            qDebug()<< e.what();
                        }
                        break;
                    }
                    case Pylon::ComponentType_Range:{
                        pcPtr = convertGrabResultToPointCloud(grabResult);
                        emit grabbedPointCloud();
                    }break;
                    case Pylon::ComponentType_Confidence:{
                        currentConfidence = QImage(component.GetWidth(), component.GetHeight(),QImage::Format_Grayscale16);
                        memcpy(currentConfidence.bits(), component.GetData(), component.GetDataSize());

                        emit grabbedConfidence();
                    }break;
                    case Pylon::ComponentType_Undefined:
                        break;
                    }}
            }
            emit grabbed();
        }catch(const Pylon::GenericException &e){
            qDebug() << "[ERROR WHILE MAKING GRABBED IMAGE]" << e.what();
        }
#endif
    }else{
        try{
            Pylon::CPylonImage image;
            image.AttachGrabResultBuffer(grabResult);
            currentImage = convertPylonImageToQImage(image);
            emit grabbed();
        }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
            Qylon::log(QString("Image conversion error occurred.") + e.what());
        }
    }
    lockImage.unlock();

    PYLON_UNUSED( camera );
}
void Qylon::Camera::OnAttached(Pylon::CInstantCamera &camera)
{
    parent->log(QString(camera.GetDeviceInfo().GetFriendlyName().c_str()) + " is attached");
    PYLON_UNUSED( camera );
}
void Qylon::Camera::OnDetached(Pylon::CInstantCamera &camera)
{
    parent->log(QString(camera.GetDeviceInfo().GetFriendlyName().c_str()) + " is detached");
    PYLON_UNUSED( camera );
}
void Qylon::Camera::OnDestroy(Pylon::CInstantCamera &camera)
{
    parent->log(QString(camera.GetDeviceInfo().GetFriendlyName().c_str()) + " will be destroyed");
    PYLON_UNUSED( camera );
}
void Qylon::Camera::OnOpened(Pylon::CInstantCamera &camera)
{
    parent->log(QString(camera.GetDeviceInfo().GetFriendlyName().c_str()) + " is opened");
    emit connected();
    PYLON_UNUSED( camera );
}
void Qylon::Camera::OnClosed(Pylon::CInstantCamera &camera)
{

    parent->log(QString(camera.GetDeviceInfo().GetFriendlyName().c_str()) + " is closed");
    PYLON_UNUSED( camera );
}
void Qylon::Camera::OnGrabStarted(Pylon::CInstantCamera &camera)
{
    parent->log(QString(camera.GetDeviceInfo().GetFriendlyName().c_str()) + " is starting to grab");
    emit grabbingState(true);
    PYLON_UNUSED( camera );
}
void Qylon::Camera::OnGrabStopped(Pylon::CInstantCamera &camera)
{
    parent->log(QString(camera.GetDeviceInfo().GetFriendlyName().c_str()) + " is stopped grabbing");
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
    parent->log(QString(camera.GetDeviceInfo().GetFriendlyName().c_str()) + " is removed");
    emit removed();

    PYLON_UNUSED( camera );
}
void Qylon::Camera::OnCameraEvent(Pylon::CInstantCamera &camera, intptr_t userProvidedId, GenApi_3_1_Basler_pylon::INode *pNode)
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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Qylon::Camera::convertGrabResultToPointCloud(const Pylon::CGrabResultPtr &grabResult)
{
    const auto container = grabResult->GetDataContainer();
    const auto rangeComponent = container.GetDataComponent(0);
    const auto intensityComponent = container.GetDataComponent(1);

    const uint32_t width = rangeComponent.GetWidth();
    const uint32_t height = rangeComponent.GetHeight();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    ptrPointCloud->width = width;
    ptrPointCloud->height = height;
    ptrPointCloud->points.resize(width * height);
    ptrPointCloud->is_dense = false; // Organized point cloud

    const auto* pSrcPoint = (const Point*)rangeComponent.GetData();
    const auto* pIntensity = (const uint16_t*)intensityComponent.GetData();

    // Find minZ and maxZ
    float minZ = std::numeric_limits<float>::max();
    float maxZ = std::numeric_limits<float>::min();

#pragma omp parallel
    {
        float local_minZ = minZ;
        float local_maxZ = maxZ;

#pragma omp for nowait
        for (uint32_t i = 0; i < height * width; ++i) {
            float z = pSrcPoint[i].z;
            if (z < local_minZ) local_minZ = z;
            if (z > local_maxZ) local_maxZ = z;
        }

#pragma omp critical
        {
            if (local_minZ < minZ) minZ = local_minZ;
            if (local_maxZ > maxZ) maxZ = local_maxZ;
        }
    }

    float rangeZ = maxZ - minZ;

#pragma omp parallel for
    for (uint32_t i = 0; i < height * width; ++i){
        pcl::PointXYZRGB& dstPoint = ptrPointCloud->points[i];

        dstPoint.x = pSrcPoint[i].x;
        dstPoint.y = pSrcPoint[i].y;
        dstPoint.z = pSrcPoint[i].z;

        // Normalize the intensity value (assuming 16-bit intensity)
        float intensity = pIntensity[i] / 65535.0f;

        // Combine distance and intensity for coloring
        float z = dstPoint.z;
        float ratio = (z - minZ) / rangeZ;

        // Apply a weight to intensity for more vivid colors
        float intensityWeight = 0.5f;  // Adjust this weight as needed
        float combinedIntensity = ratio * (1 - intensityWeight) + intensity * intensityWeight;

        dstPoint.r = static_cast<uint8_t>(255 * (1 - combinedIntensity));
        dstPoint.g = static_cast<uint8_t>(255 * (1 - fabs(2 * combinedIntensity - 1)));
        dstPoint.b = static_cast<uint8_t>(255 * combinedIntensity);
    }
    return ptrPointCloud;
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
