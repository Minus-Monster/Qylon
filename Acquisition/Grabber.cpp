#ifdef GRABBER_ENABLED
#include "Grabber.h"
#include "GrabberWidget.h"
#include "Qylon.h"

Qylon::Grabber::Grabber(Qylon *parentQylon, unsigned int boardIndex)
    : parent{parentQylon}, boardNumIndex(boardIndex)
{
    widget = new GrabberWidget(this);
    Fg_InitLibraries(nullptr);
}

bool Qylon::Grabber::loadApplet(QString file){
    // release();
    currentFg = Fg_Init(file.toStdString().c_str(), boardNumIndex);
    if(currentFg == 0){
        Qylon::log("Failed to load an applet file. " + QString(Fg_getLastErrorDescription(currentFg)));
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
    for (int i=0; i< dmaCount; i++){
        Qylon::log("Try to initialize grabber(" + QString::number(boardNumIndex) + ") with DMA " +  QString::number(i) );

        auto apcData = new APC;
        apcData->dmaIndex = i;
        apcData->address = this;
        apcData->width = getWidth(i);
        apcData->height = getHeight(i);
        int DMALength = getWidth(i)*getHeight(i) * getBytesPerPixel(i);

        apcData->memBuf = Fg_AllocMemHead(currentFg, DMALength, 1);
        apcData->imageBuffer = (unsigned short*)malloc((size_t)DMALength);
        Fg_AddMem(currentFg, apcData->imageBuffer, (size_t)DMALength, 0, apcData->memBuf);
        if(apcData->memBuf == NULL){
            Qylon::log("Failed initializing grabber(" + QString::number(boardNumIndex) + ") DMA : " + QString::number(i) + "(" + QString::number(i+1) +")");
            return;
        }

        struct FgApcControl ctrl;
        ctrl.version = 0;
        ctrl.data = apcData;
        ctrl.flags = FG_APC_DEFAULTS| FG_APC_IGNORE_STOP |  FG_APC_IGNORE_TIMEOUTS | FG_APC_HIGH_PRIORITY; //FG_APC_HIGH_PRIORITY
        ctrl.timeout = 500000;

        ctrl.func = &Grabber::CallbackFromGrabber;
        Fg_registerApcHandler(currentFg, i, &ctrl, FG_APC_CONTROL_BASIC);
        apcDataList.push_back(apcData);
    }
    Qylon::log("Finished to initialize grabber(" + QString::number(boardNumIndex) + ")");
}

void Qylon::Grabber::reitialize(int dmaCount)
{
    release();
    initialize(dmaCount);
}

int Qylon::Grabber::getBytesPerPixel(int dmaIndex)
{
    int format;
    Fg_getParameter(currentFg, FG_FORMAT, &format, dmaIndex);
    int bytesPerPixel=0;
    switch (format) {
    case FG_GRAY:
        bytesPerPixel = 1;
        break;
    case FG_GRAY16:
        bytesPerPixel = 2;
        break;
    case FG_COL24:
        bytesPerPixel = 3;
        break;
    case FG_COL32:
        bytesPerPixel = 4;
        break;
    case FG_COL30:
        bytesPerPixel = 5;
        break;
    case FG_COL48:
        bytesPerPixel = 6;
        break;
    }
    return bytesPerPixel;
}

int Qylon::Grabber::CallbackFromGrabber(frameindex_t picNr, void *ctx)
{
    auto data = reinterpret_cast<APC*>(ctx);
    Grabber* app = data->address;
    int idx = data->dmaIndex;
    dma_mem* buf = data->memBuf;

    QImage::Format imageFormat;
    switch(app->getBytesPerPixel(data->dmaIndex)){
    case 1:
        imageFormat = QImage::Format_Grayscale8;
        break;
    case 2:
        imageFormat = QImage::Format_Grayscale16;
        break;
    }
    QImage outputImage = QImage((uchar*)Fg_getImagePtrEx(app->getFg(), picNr, idx, buf),
                                app->getWidth(idx),
                                app->getHeight(idx),
                                imageFormat);
    emit app->sendImage(outputImage, idx);
    return 0;
}

Fg_Struct *Qylon::Grabber::getFg()
{
    return currentFg;
}

Qylon::APC *Qylon::Grabber::getAPC(int dmaIndex)
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
    Qylon::log("Graber(" + QString::number(boardNumIndex) + ") DMA : " +QString::number(dmaIndex) +" Continuous grabbing started.");
    auto val= Fg_AcquireEx(currentFg, dmaIndex, GRAB_INFINITE ,ACQ_STANDARD, apcDataList.at(dmaIndex)->memBuf);
    if(val != 0) Qylon::log(Fg_getErrorDescription(currentFg, val));

}

void Qylon::Grabber::sequentialGrab(int numFrame, int dmaIndex)
{
    if(currentFg == nullptr){
        Qylon::log("Grabber is not initialized");
        return;
    }
    Qylon::log("Graber(" + QString::number(boardNumIndex) + ") DMA : " +QString::number(dmaIndex) +" Sequential grabbing started.");
    auto val= Fg_AcquireEx(currentFg, dmaIndex, numFrame ,ACQ_STANDARD, apcDataList.at(dmaIndex)->memBuf);
    if(val != 0) Qylon::log(Fg_getErrorDescription(currentFg, val));
}

void Qylon::Grabber::stopGrab(int dmaIndex){
    Qylon::log("Stop grabbing Grabber(" + QString::number(boardNumIndex) + ") DMA : " + QString::number(dmaIndex));
    Fg_stopAcquireEx(currentFg, dmaIndex, apcDataList.at(dmaIndex)->memBuf, 0);
}
#endif
