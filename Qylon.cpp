
#include "Acquisition/Camera.h"
#include <Qylon.h>

void Qylon::Qylon::log(QString strings){
    qDebug() << "[QYLON]" << QTime::currentTime().toString() << ":" << strings;
}

Qylon::Qylon::Qylon(){

    qWarning() << "[NOTICE] "
                "Current bug lists : "
                "1. Why does a trigger mode switch to 'Off'?"
                "2. generateChildrenWidgetItem is called twice."
                "";

    initialize();
}

Qylon::Qylon::~Qylon(){
    for(Camera *obj : qAsConst(objectList)){
        delete obj;
    }
    Pylon::PylonTerminate();
    deleteLater();
}

void Qylon::Qylon::initialize(){
    /// pylon API initializes
    /// This function should be called only once when Qylon class generated
    /// Also Qylon should be declared only one time on the program

    Pylon::PylonAutoInitTerm initTerm;

    Pylon::PylonInitialize();
    log("Start pylon initializing");
    tlFactory = &Pylon::CTlFactory::GetInstance();

    log("Enumerating Basler devices");
    devices = new Pylon::DeviceInfoList_t;
//    tlFactory->EnumerateDevices(*devices);
}

void Qylon::Qylon::updateCameraList(){
    /// The camera list will be updated in this function
    /// All camera name(QString) are written in "cameraList"
    /// After that, "updatedCameraInformation" will be called for updating each QylonObject if available

    log("Updating available cameras list");
    auto cnt = tlFactory->EnumerateDevices(*devices);
    log("Found " + QString::number(cnt) + " Camera(s)");

    cameraList.clear();
    for(size_t i=0; i < devices->size(); i++){
        QString text = devices->at(i).GetFriendlyName().c_str();
        cameraList.push_back(text);
    }

    emit updatedCameraInformation();
}


Qylon::Camera *Qylon::Qylon::addCamera(){
    Camera *obj = new Camera(this);
    objectList.push_back(obj);

    log("new Qylon camera generated");

    return obj;
}

#ifdef GRABBER_ENABLED
Qylon::Grabber *Qylon::Qylon::addGrabber(int boardNumIndex)
{
    Grabber *obj = new Grabber(this, boardNumIndex);
    grabberList.push_back(obj);

    log("new Qylon Grabber generated");
    return obj;
}
#endif

//Qylon::QDC::QDC *Qylon::Qylon::addQDC()
//{
//    QDC::QDC *engine = new QDC::QDC;
//    log("new QDC Engined generated");

//    return engine;
//}

QStringList Qylon::Qylon::getCameraList()
{
    return cameraList;
}

const Pylon::CDeviceInfo Qylon::Qylon::getCameraIndexfromName(QString cameraName)
{
    try{
        for(unsigned int i = 0; i<devices->size(); i++){
            if(devices->at(i).GetFriendlyName().c_str() == cameraName.toStdString()) return devices->at(i);
        }
    }catch(const Pylon::GenericException &e){
        log(e.GetDescription());
    }
    return Pylon::CDeviceInfo();
}

const Pylon::CDeviceInfo Qylon::Qylon::getCameraIndexfromNumber(unsigned int number)
{
    try{
        if(number -1 > devices->size()) return Pylon::CDeviceInfo();

        return devices->at(number);

    }catch(const Pylon::GenericException &e){
        log(e.GetDescription());
    }
    return Pylon::CDeviceInfo();
}
