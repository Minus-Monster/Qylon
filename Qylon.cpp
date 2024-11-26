#include <Qylon.h>
namespace Qylon{
void Qylon::Qylon::log(QString strings){
    qDebug() << "[QYLON]" << QTime::currentTime().toString() << ":" << strings;
}

Qylon::Qylon::Qylon(){
    initialize();
}

Qylon::Qylon::~Qylon(){
#ifdef PYLON_ENABLED
    for(Camera *obj : std::as_const(cameraList)){
        delete obj;
    }
    Pylon::PylonTerminate();
#endif
    deleteLater();
}

void Qylon::Qylon::initialize(){
    /// pylon API initializes
    /// This function should be called only once when Qylon class generated
    /// Also Qylon should be declared only one time on the program
#ifdef PYLON_ENABLED
    Pylon::PylonAutoInitTerm initTerm;

    Pylon::PylonInitialize();
    log("Start pylon initializing");
    tlFactory = &Pylon::CTlFactory::GetInstance();

    log("Enumerating Basler devices");
    devices = new Pylon::DeviceInfoList_t;
//    tlFactory->EnumerateDevices(*devices);
#endif
}

#ifdef PYLON_ENABLED
void Qylon::Qylon::updateCameraList(){
    /// The camera list will be updated in this function
    /// All camera name(QString) are written in "cameraList"
    /// After that, "updatedCameraInformation" will be called for updating each QylonObject if available

    log("Updating available cameras list");
    auto cnt = tlFactory->EnumerateDevices(*devices);
    log("Found " + QString::number(cnt) + " Camera(s)");

    currentCameraList.clear();
    for(size_t i=0; i < devices->size(); i++){
        QString text = devices->at(i).GetFriendlyName().c_str();
        currentCameraList.push_back(text);
    }

    emit updatedCameraInformation();
}


Camera *Qylon::Qylon::addCamera(){
    Camera *obj = new Camera(this);
    cameraList.push_back(obj);

    log("new Qylon camera generated");

    return obj;
}
#ifdef VTOOLS_ENABLED
vTools *Qylon::Qylon::addVTools()
{
    vTools *obj = new vTools(this);
    vToolsList.push_back(obj);
    log("new Qylon vTools generated");

    return obj;
}
#endif
QStringList Qylon::Qylon::getCameraList()
{
    return currentCameraList;
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
#endif

#ifdef GRABBER_ENABLED
Grabber *Qylon::Qylon::addGrabber(int boardNumIndex)
{
    Grabber *obj = new Grabber(this, boardNumIndex);
    grabberList.push_back(obj);

    log("new Qylon Grabber generated");
    return obj;
}
#endif
}
