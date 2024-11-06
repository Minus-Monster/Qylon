#ifndef QYLON_H
#define QYLON_H
#include <QObject>
#include <QDebug>
#include <QThread>
#include <QTime>
#ifdef PYLON_ENABLED
#include "Camera/Camera.h"
#include "vTools/vTools.h"
#endif
#ifdef GRABBER_ENABLED
#include "Acquisition/Grabber.h"
#endif
//#include "Processing/QDC.h"

namespace Qylon{
class Camera;
class Grabber;
class Qylon : public QObject {
    Q_OBJECT
public:
    Qylon();
    ~Qylon();
    static void log(QString strings);
    void initialize();

    //pylon setting
#ifdef PYLON_ENABLED
    void updateCameraList();
    QStringList getCameraList();
    const Pylon::CDeviceInfo getCameraIndexfromName(QString cameraName);
    const Pylon::CDeviceInfo getCameraIndexfromNumber(unsigned int number);
    Camera *addCamera();
    vTools *addVTools();
#endif
#ifdef GRABBER_ENABLED
    Grabber *addGrabber(int boardNumIndex =0);
#endif
//    QDC::QDC *addQDC();


signals:
    void updatedCameraInformation();

private:
    //pylon setting
#ifdef PYLON_ENABLED
    Pylon::CTlFactory *tlFactory;
    Pylon::DeviceInfoList_t *devices = nullptr;
    QList<Camera*> cameraList;
    QList<vTools*> vToolsList;
    QStringList currentCameraList;
#endif
#ifdef GRABBER_ENABLED
    QList<Grabber*> grabberList;
#endif
};
}


#endif // QYLON_H
