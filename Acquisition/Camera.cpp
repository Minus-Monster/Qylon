#include "Qylon.h"
#include "Camera.h"
#include "CameraWidget.h"

#ifdef PCL_ENABLED
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#endif


Qylon::Camera::Camera(Qylon *parentQylon) : parent(parentQylon){
    widget = new CameraWidget(this);
    connect(this, &Camera::grabbingState, widget, [=](bool grabbing){
        // should have sending a message that grabbing is started
        emit widget->grabbingState(grabbing);

    });
    connect(this, &Camera::removed, widget, &CameraWidget::disconnectCamera);
    connect(this, &Camera::connected, widget, &CameraWidget::connectedCameraFromOutside);

    // The order of these codes is critically importatnt
    currentInstantCamera.RegisterImageEventHandler(this, Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_None);
    currentInstantCamera.RegisterConfiguration(new Pylon::CAcquireContinuousConfiguration(), Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_Delete );
    currentInstantCamera.RegisterConfiguration(this, Pylon::RegistrationMode_Append, Pylon::Cleanup_None );
}
#ifdef PCL_ENABLED
struct Point{
    float x;
    float y;
    float z;
};
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Qylon::Camera::convertGrabResultToPointCloud(const Pylon::CGrabResultPtr &grabResult)
{
    // An organized point cloud is used, i.e., for each camera pixel there is an entry
    // in the data structure indicating the 3D coordinates calculated from that pixel.
    // If the camera wasn't able to create depth information for a pixel, the x, y, and z coordinates
    // are set to NaN. These NaNs will be retained in the PCL point cloud.

    // Allocate PCL point cloud.
    const auto container = grabResult->GetDataContainer();
    const auto rangeComponent = container.GetDataComponent(0);
    const auto intensityComponent = container.GetDataComponent(1);

    const size_t width = rangeComponent.GetWidth();
    const size_t height = rangeComponent.GetHeight();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    ptrPointCloud->width = (uint32_t)width;
    ptrPointCloud->height = (uint32_t)height;
    ptrPointCloud->points.resize(width * height);
    ptrPointCloud->is_dense = false; // Organized point cloud

    // Create a pointer to the 3D coordinates of the first point.
    // grabResult[0] always refers to the point cloud data.
    const auto* pSrcPoint = (const Point*)rangeComponent.GetData();

    // Create a pointer to the intensity information, stored in the second buffer part.
    const auto* pIntensity = (const uint16_t*)intensityComponent.GetData();


    //    cv::Mat range = cv::Mat(rangeComponent.GetHeight(), rangeComponent.GetWidth(), CV_32FC3, (void*)rangeComponent.GetData());

    // Set the points.
    for (size_t i = 0; i < height * width; ++i, ++pSrcPoint, ++pIntensity)
    {
        // Set the x/y/z coordinates.
        pcl::PointXYZRGB& dstPoint = ptrPointCloud->points[i];

        dstPoint.x = pSrcPoint->x;
        dstPoint.y = pSrcPoint->y;
        dstPoint.z = pSrcPoint->z;

        // Use the intensity value of the pixel for coloring the point.
        dstPoint.r = dstPoint.g = dstPoint.b = (uint8_t)(*pIntensity >> 8);

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
        parent->log(e.GetDescription());
    }catch (QString error){
        parent->log(error);
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
    catch(const Pylon::GenericException &e){ parent->log(QString::fromStdString(e.GetDescription()));}
    catch(QString e){ parent->log(e);}

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
    }catch(const QString error){ parent->log("Sequential Grabbing Failed. " + error);}
    catch(const Pylon::GenericException &e){ parent->log(("Sequential Grabbing Failed. " + QString::fromStdString(e.GetDescription())));}
}

void Qylon::Camera::continuousGrab()
{
    try{
        if(!this->isOpened()) throw QString("Camera is not opened.");
        currentInstantCamera.AcquisitionMode.TrySetValue(Basler_UniversalCameraParams::AcquisitionMode_Continuous);
        currentInstantCamera.StartGrabbing(Pylon::GrabStrategy_OneByOne, Pylon::GrabLoop_ProvidedByInstantCamera);
    }catch(const QString error){ parent->log("Continuous Grabbing Failed. " + error);}
    catch(const Pylon::GenericException &e){ parent->log(("Continuous Grabbing Failed. " + QString::fromStdString(e.GetDescription())));}
}

void Qylon::Camera::stopGrab()
{
    try{
        if(!this->isOpened()) throw QString("Camera is not opened.");
        currentInstantCamera.StopGrabbing();
    }catch(const Pylon::GenericException &e){ parent->log(QString::fromStdString(e.GetDescription()));}
}

bool Qylon::Camera::isOpened(){
    return currentInstantCamera.IsOpen();
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
            // PCL Code

            pcPtr.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
            Point* srcPoint;
            uint16_t* srcIntensity;
            size_t width;
            size_t height;
            cv::Mat range;
            auto container = grabResult->GetDataContainer();
            for(size_t i = 0; i < container.GetDataComponentCount(); ++i){
                auto component = container.GetDataComponent(i);
                if(component.IsValid()){
                    switch (component.GetComponentType()){
                    case Pylon::ComponentType_Intensity:{
                        try{
                            currentIntensity = QImage((const uchar *)component.GetData(), component.GetWidth(), component.GetHeight(), QImage::Format_Grayscale16);
                            //Point Cloud
                            srcIntensity = (uint16_t*)component.GetData();
                            emit grabbedIntensity();

                        }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
                            qDebug()<< e.what();
                        }
                        break;
                    }
                    case Pylon::ComponentType_Range:{
                        // Qt doesn't have any format to describe for 3 channel float type
                        range = cv::Mat(component.GetHeight(), component.GetWidth(), CV_32FC3, (void*)component.GetData());
                        // Point Cloud
                        srcPoint = (Point*)component.GetData();
                        width = component.GetWidth();
                        height = component.GetHeight();
                        pcPtr->width = (uint32_t)width;
                        pcPtr->height = (uint32_t)height;
                        pcPtr->points.resize(width*height);
                        pcPtr->is_dense = false;

                        //                        currentImage = QImage(range.cols, range.rows, QImage::Format_RGB888);
                        //                        for (int i=0; i< range.rows; ++i){
                        //                            uchar *p = currentImage.scanLine(i);
                        //                            const float *in = range.ptr<float>(i);
                        //                            for(int j=0; j < range.cols; ++j){
                        //                                float b = std::max(0.0f, std::min(in[j*3]*255.f, 255.f));
                        //                                float g = std::max(0.0f, std::min(in[j*3+1]*255.f, 255.f));
                        //                                float r = std::max(0.0f, std::min(in[j*3+2]*255.f, 255.f));
                        //                                p[j*3] = static_cast<uchar>(b);
                        //                                p[j*3+1] = static_cast<uchar>(g);
                        //                                p[j*3+2] = static_cast<uchar>(r);
                        //                            }
                        //                        }
                        //                        cv::imshow("Range", range);

                        emit grabbedPointCloud();
                        break;
                    }
                    case Pylon::ComponentType_Confidence:
                        currentConfidence = QImage((const uchar *)component.GetData(), component.GetWidth(), component.GetHeight(), QImage::Format_Grayscale16);
                        emit grabbedConfidence();

                        break;
                    case Pylon::ComponentType_Undefined:
                        break;
                    }}
            }
            //PointCloud
            for(size_t i=0; i< height*width; ++i, ++srcPoint, ++srcIntensity){
                pcl::PointXYZRGB& dstPoint = pcPtr->points[i];
                dstPoint.x = srcPoint->x;
                dstPoint.y = srcPoint->y;
                dstPoint.z = srcPoint->z;

                dstPoint.r = dstPoint.g = dstPoint.b = (uint8_t)(*srcIntensity >> 8);
            }

            //            pcPtr = convertGrabResultToPointCloud(grabResult);
            emit grabbedPointCloud();

        }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
            qDebug() << "[ERROR WHILE MAKING GRABBED IMAGE]" << e.what();
        }
#endif
    }else{
        try{
            // Initializing QImage type
            QString currentPixelFormat = static_cast<GenApi_3_1_Basler_pylon::CEnumerationPtr>(camera.GetNodeMap().GetNode("PixelFormat"))->ToString().c_str();

            if(currentPixelFormat == "Mono8"){
                currentImage = QImage( grabResult->GetWidth(), grabResult->GetHeight(), QImage::Format_Grayscale8);
                currentBuffer = grabResult->GetBuffer();

                formatConverter.OutputPixelFormat = Pylon::PixelType_Mono8;
            }else if(currentPixelFormat == "Mono12" || currentPixelFormat == "Mono16" ){
                currentImage = QImage( grabResult->GetWidth(), grabResult->GetHeight(), QImage::Format_Grayscale16);
                currentBuffer = grabResult->GetBuffer();

                formatConverter.OutputPixelFormat = Pylon::PixelType_Mono16;
            }
            else{
                currentImage = QImage( grabResult->GetWidth(), grabResult->GetHeight(), QImage::Format_RGB32 );
                currentBuffer = grabResult->GetBuffer();

                formatConverter.OutputPixelFormat = Pylon::PixelType_BGRA8packed;
            }
            formatConverter.Convert(currentImage.bits(), currentImage.sizeInBytes(), grabResult);


        }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
            //        qDebug() << "hEre fuck " << e.what();
        }
        emit grabbed();
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
