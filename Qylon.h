#ifndef QYLON_H
#define QYLON_H
#include <QObject>
#include <QDebug>
#include <QThread>
#include <QTime>

#include "Acquisition/Camera.h"
#include "Processing/vTools.h"
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
    void updateCameraList();

    //pylon setting
    QStringList getCameraList();
    const Pylon::CDeviceInfo getCameraIndexfromName(QString cameraName);
    const Pylon::CDeviceInfo getCameraIndexfromNumber(unsigned int number);
    Camera *addCamera();
    vTools *addVTools();
#ifdef GRABBER_ENABLED
    Grabber *addGrabber(int boardNumIndex =0);
#endif
//    QDC::QDC *addQDC();


signals:
    void updatedCameraInformation();

private:

    //pylon setting
    Pylon::CTlFactory *tlFactory;
    Pylon::DeviceInfoList_t *devices = nullptr;
    QList<Camera*> objectList;
    QList<Grabber*> grabberList;
    QStringList cameraList;
};
}


#endif // QYLON_H
