#ifdef GRABBER_ENABLED
#include "Grabber.h"
#include "GrabberWidget.h"
#include "Qylon.h"

extern int ApcFunc(frameindex_t picNr, void *ctx){
    auto data = reinterpret_cast<struct Qylon::fg_apc_data*>(ctx);
    QImage image((unsigned char*)Fg_getImagePtrEx(data->address->getFg(), picNr, data->dmaIndex, data->memBuf),
                 data->width, data->height, QImage::Format_Grayscale8);

    emit data->address->sendImage(image);
    return 0;
}

Qylon::Grabber::Grabber(Qylon *parentQylon, unsigned int boardIndex)
    : parent{parentQylon}, boardNumIndex(boardIndex)
{
    widget = new GrabberWidget(this);
}

bool Qylon::Grabber::loadApplet(QString file){
    release();
    currentFg = Fg_Init(file.toStdString().c_str(), boardNumIndex);
    if(currentFg == 0){
        Qylon::log("Failed to load an applet file." + QString(Fg_getLastErrorDescription(currentFg)));
        return false;
    }
    Qylon::log(file + " is loaded.");
    return true;
}

bool Qylon::Grabber::loadConfiguration(QString file){
    if(currentFg == nullptr){
        Qylon::log("Grabber is not initialized.");
        return false;
    }
    auto error = Fg_loadConfig(currentFg, file.toStdString().c_str());
    if(0 > error){
        Qylon::log("Grabber couldn't load a configuration file. " + QString(Fg_getErrorDescription(currentFg, error)));
        return false;
    }
    Qylon::log(file + " is loaded");
    return true;
}

int Qylon::Grabber::getWidth(int dmaIndex){
    int value;
    Fg_getParameter(currentFg, FG_WIDTH, (void*)&value, dmaIndex);
    return value;
}

int Qylon::Grabber::getHeight(int dmaIndex){
    int value;
    Fg_getParameter(currentFg, FG_HEIGHT, (void*)&value, dmaIndex);
    return value;
}

void Qylon::Grabber::setParameterValue(QString typeName, int value, int dmaIndex){
    Fg_setParameterWithType(currentFg, Fg_getParameterIdByName(currentFg, typeName.toStdString().c_str()), value, dmaIndex);
}

int Qylon::Grabber::getParameterValue(QString typeName, int dmaIndex){
    int value;
    return Fg_getParameterWithType(currentFg, Fg_getParameterIdByName(currentFg, typeName.toStdString().c_str()), &value, dmaIndex);
}

void Qylon::Grabber::initialize(int dmaCount){
    currentDmaCount = dmaCount;
    Qylon::log("Try to initialize the grabber with " + QString::number(dmaCount) + " DMA.");
    for (int i=0; i< dmaCount; i++){
        auto apcData = new fg_apc_data;

        apcData->dmaIndex = i;
        apcData->address = this;
        apcData->width = getWidth(i);
        apcData->height = getHeight(i);
        apcData->memBuf = Fg_AllocMemEx(currentFg, (getWidth(i)*getHeight(i)*BUF_NUM), BUF_NUM);
        if(apcData->memBuf == NULL){
            Qylon::log("Failed initializing the grabber. DMA : " + QString::number(i) + "(" + QString::number(i+1) +")");
            return;
        }
        apcData->ctrl.version = 0;
        apcData->ctrl.data = &apcData[i];
        apcData->ctrl.func = ApcFunc;
        apcData->ctrl.flags = FG_APC_DEFAULTS| FG_APC_IGNORE_STOP |  FG_APC_IGNORE_TIMEOUTS; //FG_APC_HIGH_PRIORITY
        apcData->ctrl.timeout = 500000;
        Fg_registerApcHandler(currentFg, i, &apcData->ctrl, FG_APC_CONTROL_BASIC);
        apcDataList.push_back(apcData);
    }
    Qylon::log("Finished to initialize the grabber.");
}

void Qylon::Grabber::reitialize(int dmaCount)
{
    release();
    initialize(dmaCount);
}

Fg_Struct *Qylon::Grabber::getFg()
{
    return currentFg;
}

Qylon::fg_apc_data *Qylon::Grabber::getAPC(int dmaIndex)
{
    return apcDataList.at(dmaIndex);
}

QWidget *Qylon::Grabber::getWidget()
{
    return widget;
}

void Qylon::Grabber::release(){
    Qylon::log("Release all grabber memory.");
    if(currentFg == nullptr) return;

    for(int i=0; i<currentDmaCount; i++){
        Fg_FreeMemEx(currentFg, apcDataList.at(i)->memBuf);
        apcDataList.removeAt(i);
    }
    Fg_FreeGrabber(currentFg);
    currentFg = nullptr;
}

void Qylon::Grabber::singleGrab(int dmaIndex)
{
    if(currentFg == nullptr){
        Qylon::log("Grabber is not initialized");
        return;
    }
    Qylon::log("Try to capture an one shot.");
    auto val = Fg_AcquireEx(currentFg, dmaIndex, 1 ,ACQ_STANDARD, apcDataList.at(dmaIndex)->memBuf);
    if(val != 0) Qylon::log(Fg_getErrorDescription(currentFg, val));
}

void Qylon::Grabber::continuousGrab(int dmaIndex){
    if(currentFg == nullptr){
        Qylon::log("Grabber is not initialized");
        return;
    }
    Qylon::log("DMA : " +QString::number(dmaIndex) +" Continuous grabbing started.");
    auto val= Fg_AcquireEx(currentFg, dmaIndex, GRAB_INFINITE ,ACQ_STANDARD, apcDataList.at(dmaIndex)->memBuf);
    if(val != 0) Qylon::log(Fg_getErrorDescription(currentFg, val));

}

void Qylon::Grabber::sequentialGrab(int numFrame, int dmaIndex)
{
    if(currentFg == nullptr){
        Qylon::log("Grabber is not initialized");
        return;
    }
    Qylon::log("DMA : " +QString::number(dmaIndex) +" Sequential grabbing started.");
    auto val= Fg_AcquireEx(currentFg, dmaIndex, numFrame ,ACQ_STANDARD, apcDataList.at(dmaIndex)->memBuf);
    if(val != 0) Qylon::log(Fg_getErrorDescription(currentFg, val));
}

void Qylon::Grabber::stopGrab(int dmaIndex){
    Qylon::log("Stop grabbing.");
    Fg_stopAcquireEx(currentFg, dmaIndex, apcDataList.at(dmaIndex)->memBuf, 0);
}
#endif
