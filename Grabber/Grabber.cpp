#include <thread>
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

int Qylon::Grabber::CallbackFromGrabber(frameindex_t picNr, void *ctx)
{
    auto data = reinterpret_cast<APC*>(ctx);
    Grabber* app = data->address;
    int idx = data->dmaIndex;
    dma_mem* buf = data->memBuf;

    QImage::Format imageFormat;
    switch(app->getBytesPerPixel(data->dmaIndex)){
    case 1: imageFormat = QImage::Format_Grayscale8; break;
    case 2: imageFormat = QImage::Format_Grayscale16; break;
    }
    QImage outputImage = QImage((uchar*)Fg_getImagePtrEx(app->getFg(), picNr, idx, buf),
                                app->getWidth(idx),
                                app->getHeight(idx),
                                imageFormat);
    emit app->sendImage(outputImage, idx);
    return 0;
}

bool Qylon::Grabber::loadApplet(QString file){
    currentFg = Fg_Init(file.toStdString().c_str(), boardNumIndex);
    if(file.isEmpty()){
        Qylon::log("No path");
        return false;
    }
    if(currentFg == 0){
        Qylon::log("Failed to load an applet file. " + QString(Fg_getLastErrorDescription(currentFg)));
        return false;
    }
    Qylon::log(file + " is loaded.");
    emit loadedApplet(file);
    return true;
}

bool Qylon::Grabber::loadConfiguration(QString file, bool ignoringError){
    // Deu to the specific bug on VA, This function should not be working properly.
    if(file.isEmpty()){
        Qylon::log("No path");
        return false;
    }
    if(currentFg == nullptr){
        Qylon::log("Grabber is not initialized.");
        return false;
    }
    auto error = Fg_loadConfig(currentFg, file.toStdString().c_str());
    if(0 > error){
        Qylon::log("Grabber couldn't load a configuration file. " + QString(Fg_getErrorDescription(currentFg, error)));
        if(!ignoringError) return false;
        else Qylon::log("Ignoring the error handling policy.");
    }
    Qylon::log(file + " is loaded.");
    emit loadedConfig(file);
    return true;
}

int Qylon::Grabber::getDMACount()
{
    int val=0;
    Fg_getParameterPropertyWithType(currentFg, FG_NR_OF_DMAS, FgProperty::PROP_ID_VALUE, &val);
    return val;
}

int Qylon::Grabber::getWidth(int dmaIndex){
    int value=0;
    Fg_getParameter(currentFg, FG_WIDTH, (void*)&value, dmaIndex);
    return value;
}

int Qylon::Grabber::getHeight(int dmaIndex){
    int value=0;
    Fg_getParameter(currentFg, FG_HEIGHT, (void*)&value, dmaIndex);
    return value;
}

int Qylon::Grabber::getX(int dmaIndex)
{
    int value;
    if(!(Fg_getParameter(currentFg, FG_XOFFSET, (void*)&value, dmaIndex)== FG_OK)){
        return 0;
    }
    return value;
}

int Qylon::Grabber::getY(int dmaIndex)
{
    int value=0;
    if(!(Fg_getParameter(currentFg, FG_YOFFSET, (void*)&value, dmaIndex)== FG_OK)){
        return 0;
    }
    return value;
}


QString Qylon::Grabber::getParameterStringValue(QString typeName, int dmaIndex)
{
    std::string buf;
    Fg_getParameterWithType(currentFg, Fg_getParameterIdByName(currentFg, typeName.toStdString().c_str()), buf, dmaIndex);
    return QString::fromStdString(buf);
}

int Qylon::Grabber::getParameterIntValue(QString typeName, int dmaIndex)
{
    int val;
    Fg_getParameterWithType(currentFg, Fg_getParameterIdByName(currentFg, typeName.toStdString().c_str()), &val, dmaIndex);
    return val;
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

void Qylon::Grabber::setParameterValue(QString typeName, int value, int dmaIndex){
    Fg_setParameterWithType(currentFg, Fg_getParameterIdByName(currentFg, typeName.toStdString().c_str()), value, dmaIndex);
    emit updatedParametersValue();

    Qylon::log(QString("DMA:" + QString::number(dmaIndex) + " " + typeName + " tries to be set to "
                       + QString::number(value) + " (Result:" + QString::number(getParameterIntValue(typeName, dmaIndex)) +")"));
}

void Qylon::Grabber::setParameterValue(QString typeName, QString value, int dmaIndex){
    Fg_setParameterWithType(currentFg, Fg_getParameterIdByName(currentFg, typeName.toStdString().c_str()), value.toStdString().c_str(), dmaIndex);
    emit updatedParametersValue();
    Qylon::log(QString("DMA:" + QString::number(dmaIndex) + " " + typeName + " tries to be set to "
                       + value + " (Result:" + getParameterStringValue(typeName, dmaIndex) + ")"));
}

void Qylon::Grabber::setParameterValue(int typeNum, int value, int dmaIndex){
    Fg_setParameter(currentFg, typeNum, &value, dmaIndex);
    emit updatedParametersValue();
    QString typeName= Fg_getParameterNameById(currentFg, typeNum, dmaIndex);
    Qylon::log(QString("DMA:" + QString::number(dmaIndex) + " " + typeName + " tries to be set to "
                       + QString::number(value) + " (Result:" + QString::number(getParameterIntValue(typeName, dmaIndex)) + ")"));
}

bool Qylon::Grabber::registerAPCHandler(int imgBufCnt){
    try{
        imageBufferSize = imgBufCnt;
        for (int i=0; i< getDMACount(); ++i){
            Qylon::log("Try to register APC handler to grabber(" + QString::number(boardNumIndex) + ") with DMA " +  QString::number(i) + ". It will include " + QString::number(imgBufCnt) + " image buffer(s)."  );

            auto apcData = new APC;
            struct FgApcControl ctrl;
            ctrl.version = 0;
            ctrl.data = apcData;
            ctrl.flags = FG_APC_DEFAULTS| FG_APC_IGNORE_STOP |  FG_APC_IGNORE_TIMEOUTS | FG_APC_HIGH_PRIORITY; //FG_APC_HIGH_PRIORITY
            ctrl.timeout = 500000;

            ctrl.func = &Grabber::CallbackFromGrabber;
            auto result = Fg_registerApcHandler(currentFg, i, &ctrl, FG_APC_CONTROL_BASIC);
            if(result == FG_OK) apcDataList.push_back(apcData);
            else{
                delete apcData;
                throw result;
            }
        }
        Qylon::log("Finished to register APC handler to grabber(" + QString::number(boardNumIndex) + ")");
        return true;
    }catch(...){
        Qylon::log("Failed to register APC handler to grabber. " + QString(Fg_getLastErrorDescription(currentFg)));
        return false;
    }
}

void Qylon::Grabber::setImage(const QImage image, int dmaIndex)
{
    // If grabber is not grabbing, ignoring this code below.
    if(getDMACount() < 2) return;
    APC *currentAPC = getAPC(dmaIndex); // DMA 1 is for the output
    if(!currentAPC->allocated) return;
    if(currentAPC->width*currentAPC->height != image.width()*image.height() * image.bitPlaneCount()/8){
        qDebug() << "Doesn't match the memory size of the image";
        return;
    }
    int cnt = imagePushCnt++%imageBufferSize;
    memcpy(currentAPC->imageBuffer.at(cnt), image.bits(), image.sizeInBytes());
    if((image.sizeInBytes() % 32) != 0) qDebug() << "Image size should be zero when be divided by 32. Current value is" << image.sizeInBytes();

    int r = Fg_queueBuffer(currentFg, cnt, image.sizeInBytes(), 1, currentAPC->memBuf);
    auto error = Fg_getErrorDescription(currentFg, r);
    if(QString(error) != QString("No error")) Qylon::log(QString(QString::number(r) + ":") +error);
    auto frameNum = Fg_getLastPicNumberBlockingEx(currentFg, cnt, 1, 1, currentAPC->memBuf);
    if(QString(Fg_getLastErrorDescription(currentFg)) == "Transfer not active"){
        qDebug() << "Transfer error";
    }
    if(QString(Fg_getLastErrorDescription(currentFg)) == "Timeout"){
        Qylon::log("Timeout error occurred.");
    }
}

void Qylon::Grabber::setBuffer(unsigned short *buffer, int width, int height, int depth, int dmaIndex)
{
    // If grabber is not grabbing, ignoring this code below.
    if(getDMACount() < 2) return;
    APC *currentAPC = getAPC(dmaIndex); // DMA 1 is for the output
    if(!currentAPC->allocated) return;
    int imageSize = width*height*depth/8;
    if(currentAPC->width*currentAPC->height != imageSize){
        qDebug() << "Doesn't match the memory size of the image";
        return;
    }
    int cnt = imagePushCnt++%imageBufferSize;
    if((imageSize % 32) != 0) qDebug() << "Image size should be zero when be divided by 32. Current value is" << imageSize;
    memcpy(currentAPC->imageBuffer.at(cnt), buffer, imageSize);

    int r = Fg_queueBuffer(currentFg, cnt, imageSize, 1, currentAPC->memBuf);
    auto error = Fg_getErrorDescription(currentFg, r);
    if(QString(error) != QString("No error")) Qylon::log(QString(QString::number(r) + ":") +error);
    // auto frameNum = Fg_getLastPicNumberBlockingEx(currentFg, cnt, 1, 1, currentAPC->memBuf);
    // if(QString(Fg_getLastErrorDescription(currentFg)) == "Transfer not active"){
    //     Qylon::log("Transferring error occurred on grabber in frame(" + QString::number(frameNum) + ")");
    // }
    // if(QString(Fg_getLastErrorDescription(currentFg)) == "Timeout"){
    //     Qylon::log("Timeout error occurred on grabber in frame(" + QString::number(frameNum) + ")");
    // }
}


Fg_Struct *Qylon::Grabber::getFg()
{
    return currentFg;
}

Qylon::Grabber::APC *Qylon::Grabber::getAPC(int dmaIndex)
{
    if(apcDataList.isEmpty()) return nullptr;
    return apcDataList.at(dmaIndex);
}

int Qylon::Grabber::getLastFrameCount(int dmaIndex)
{
    return Fg_getLastPicNumberEx(currentFg, dmaIndex, getAPC(dmaIndex)->memBuf);
}

QWidget *Qylon::Grabber::getWidget()
{
    return widget;
}

void Qylon::Grabber::release(){
    Qylon::log("Release all grabber's memories.");
    if(currentFg == nullptr) return;
    Fg_FreeGrabber(currentFg);
    currentFg = nullptr;
    emit released();

    if(apcDataList.size() == 0) return;
    for(int i=0; i<getDMACount(); i++){
        if(apcDataList.at(i)->allocated)
            Fg_FreeMemEx(currentFg, apcDataList.at(i)->memBuf);
    }
    apcDataList.clear();
}

void Qylon::Grabber::allocateImageBuffer(int dmaIndex)
{
    apcDataList.at(dmaIndex)->dmaIndex = dmaIndex;
    apcDataList.at(dmaIndex)->address = this;
    apcDataList.at(dmaIndex)->width = getWidth(dmaIndex);
    apcDataList.at(dmaIndex)->height = getHeight(dmaIndex);
    int DMALength = getWidth(dmaIndex)*getHeight(dmaIndex) *getBytesPerPixel(dmaIndex);

    apcDataList.at(dmaIndex)->memBuf = Fg_AllocMemHead(currentFg, DMALength * imageBufferSize, imageBufferSize);
    for(int bufNum=0; bufNum < imageBufferSize; ++bufNum){
        apcDataList.at(dmaIndex)->imageBuffer.append((unsigned short*)malloc((size_t)DMALength));
        auto subBufferIdx = Fg_AddMem(currentFg, apcDataList.at(dmaIndex)->imageBuffer[bufNum], (size_t)DMALength, bufNum, apcDataList.at(dmaIndex)->memBuf);
    }
    Qylon::log("DMA:"+ QString::number(dmaIndex)+ " Memory allocated with image buffer(" +QString::number(imageBufferSize)+ ")");
    apcDataList.at(dmaIndex)->allocated = true;
}

void Qylon::Grabber::deallocateImageBuffer(int dmaIndex)
{
    if(!apcDataList.at(dmaIndex)->allocated) return;
    for(int i=0; i<imageBufferSize; ++i){
        int val = Fg_DelMem(currentFg, apcDataList.at(dmaIndex)->memBuf, i);
        if(val == FG_OK){
            free(apcDataList.at(dmaIndex)->imageBuffer[i]);
            apcDataList.at(dmaIndex)->imageBuffer[i] = nullptr;
        }
    }
    apcDataList.at(dmaIndex)->imageBuffer.clear();

    auto val = Fg_FreeMemHead(currentFg, apcDataList.at(dmaIndex)->memBuf);
    apcDataList.at(dmaIndex)->memBuf = nullptr;
    if(val == FG_OK){
    }else if(val == FG_NOT_INIT){
        Qylon::log("Memory deallocation failed.");
    }else if(val == FG_STILL_ACTIVE){
        Qylon::log("Memory deallocation failed. It has opened.");
    }
    Qylon::log("Memory deallocation succeed with image buffer(" + QString::number(imageBufferSize)+")");
    apcDataList.at(dmaIndex)->allocated = false;
}

bool Qylon::Grabber::saveCurrentConfig(QString fileName)
{
    if(Fg_saveConfig(currentFg, fileName.toStdString().c_str()) != FG_OK){
        return false;
    }else return true;
}

void Qylon::Grabber::singleGrab(int dmaIndex)
{
    if(currentFg == nullptr){
        Qylon::log("Grabber is not initialized");
        return;
    }
    Qylon::log("Try to capture an one shot.");
    allocateImageBuffer(dmaIndex);
    auto val = Fg_AcquireEx(currentFg, dmaIndex, 1 ,ACQ_STANDARD, apcDataList.at(dmaIndex)->memBuf);
    if(val != 0) Qylon::log(Fg_getErrorDescription(currentFg, val));
}

void Qylon::Grabber::continuousGrab(int dmaIndex){
    if(currentFg == nullptr){
        Qylon::log("Grabber is not initialized");
        return;
    }
    Qylon::log("Graber(" + QString::number(boardNumIndex) + ") DMA : " +QString::number(dmaIndex) +" Continuous grabbing started.");
    allocateImageBuffer(dmaIndex);
    auto val= Fg_AcquireEx(currentFg, dmaIndex, GRAB_INFINITE ,ACQ_STANDARD, apcDataList.at(dmaIndex)->memBuf);
    if(val != 0){
        stopGrab(dmaIndex);
        Qylon::log(Fg_getErrorDescription(currentFg, val));
        return;
    }
    emit grabbingState(true);
}

void Qylon::Grabber::sequentialGrab(int numFrame, int dmaIndex)
{
    if(currentFg == nullptr){
        Qylon::log("Grabber is not initialized");
        return;
    }
    Qylon::log("Graber(" + QString::number(boardNumIndex) + ") DMA : " +QString::number(dmaIndex) +" Sequential grabbing started.");
    allocateImageBuffer(dmaIndex);
    auto val= Fg_AcquireEx(currentFg, dmaIndex, numFrame ,ACQ_STANDARD, apcDataList.at(dmaIndex)->memBuf);
    if(val != 0){
        stopGrab(dmaIndex);
        Qylon::log(Fg_getErrorDescription(currentFg, val));
        return;
    }
}

void Qylon::Grabber::stopGrab(int dmaIndex){
    Qylon::log("Stop grabbing Grabber(" + QString::number(boardNumIndex) + ") DMA : " + QString::number(dmaIndex));
    auto val = Fg_stopAcquireEx(currentFg, dmaIndex, apcDataList.at(dmaIndex)->memBuf, 0);
    if(val !=0){
        Qylon::log(Fg_getErrorDescription(currentFg, val));
    }
    deallocateImageBuffer(dmaIndex);
    emit grabbingState(false);
}

void Qylon::Grabber::stopGrabAll()
{
    for(int i=0; i<getDMACount(); ++i){
        stopGrab(i);
    }
}

void Qylon::Grabber::grabThreadLoop(int numFrame, int dmaIndex){
    if(currentFg ==nullptr) return;

    // Waiting thread ends
    if(memHandle != nullptr) Fg_FreeMemEx(currentFg, memHandle);
    int DMALength = getWidth(dmaIndex)*getHeight(dmaIndex) *getBytesPerPixel(dmaIndex);
    memHandle = Fg_AllocMemEx(currentFg, DMALength, imageBufferSize);
    if(memHandle != NULL){
        {
            std::lock_guard<std::mutex> lock(grabberMutex);
            stopFlags[dmaIndex] = false;
        }

        std::thread([=](){
            int cnt = 0;
            if(Fg_AcquireEx(currentFg, dmaIndex, GRAB_INFINITE, ACQ_STANDARD, memHandle) == FG_OK){
                Qylon::log("Start grabbing with threading." + ((numFrame!=0) ? " Expected frames:" + QString::number(numFrame) : " Continuous mode."));
                emit grabbingState(true);
                while(!stopFlags[dmaIndex] && (cnt = Fg_getLastPicNumberBlockingEx(currentFg, cnt + 1, dmaIndex, 1000, memHandle))){
                    if(cnt < 0 ){
                        Qylon::log("Terminated.");
                        return;
                    }
                    if(numFrame !=0){
                        if(cnt > numFrame) break;
                    }
                    QImage::Format imageFormat;
                    switch(getBytesPerPixel(dmaIndex)){
                    case 1: imageFormat = QImage::Format_Grayscale8; break;
                    case 2: imageFormat = QImage::Format_Grayscale16; break;
                    default: qDebug() << "Result is here" << getBytesPerPixel(dmaIndex) << dmaIndex;
                    }
                    auto width = getWidth(dmaIndex);
                    auto height = getHeight(dmaIndex);
                    QImage output = QImage((uchar*)Fg_getImagePtrEx(currentFg, cnt, dmaIndex, memHandle),
                                           width, height, imageFormat);
                    emit sendImage(output, dmaIndex);
                }
                Fg_stopAcquireEx(currentFg, dmaIndex, memHandle, 0);
                emit grabbingState(false);
                Qylon::log("Finished grabbing in thread mode. Acquired frames:" + QString::number(cnt-1));
            }
        }).detach();
    }
}

void Qylon::Grabber::stopThreadLoop(int dmaIndex){
    std::lock_guard<std::mutex> lock(grabberMutex);
    if (stopFlags.find(dmaIndex) != stopFlags.end()) {
        stopFlags[dmaIndex] = true;
        Qylon::log("Requested to stop grabbing thread for dmaIndex: " + QString::number(dmaIndex));
    }

}

bool Qylon::Grabber::saveImage(QString dir, QString fileName,int numFrame, int dmaIndex)
{
    if(currentFg ==nullptr) return false;

    // Waiting thread ends
    if(memHandle != nullptr) Fg_FreeMemEx(currentFg, memHandle);
    int DMALength = getWidth(dmaIndex)*getHeight(dmaIndex) *getBytesPerPixel(dmaIndex);
    dma_mem *memHandle = Fg_AllocMemEx(currentFg, DMALength, imageBufferSize);
    if(memHandle != NULL){
        {
            std::lock_guard<std::mutex> lock(grabberMutex);
            stopFlags[dmaIndex] = false;
        }

        std::thread([=](){
            int cnt = 0;
            int targetFrame= 0;
            if(Fg_AcquireEx(currentFg, dmaIndex, GRAB_INFINITE, ACQ_STANDARD, memHandle) == FG_OK){
                Qylon::log("Start grabbing with save mode." + ((numFrame!=0) ? " Expected frames:" + QString::number(numFrame) : " Continuous mode."));
                emit grabbingState(true);
                while((cnt = Fg_getLastPicNumberBlockingEx(currentFg, cnt + 1, dmaIndex, 1000, memHandle))){
                    if(cnt < 0 ){
                        Qylon::log("Terminated.");
                        return;
                    }
                    if(targetFrame == numFrame) break;
                    auto buffer = Fg_getImagePtrEx(currentFg, cnt, dmaIndex, memHandle);
                    auto width = getWidth(dmaIndex);
                    auto height = getHeight(dmaIndex);

                    uint bit=getBytesPerPixel(dmaIndex)*8;

                    QFileInfo fileInfo(fileName);
                    QString cleanDir = QDir::cleanPath(dir);
                    QString filePath = QDir(cleanDir).filePath(fileInfo.completeBaseName() + QString::number(targetFrame) +".tiff");
                    // save function
                    int r = IoWriteTiff(filePath.toStdString().c_str(), (uchar*)buffer, width, height, bit, 1);
                    if(r==FG_OK){
                        qDebug() << cnt;
                        ++targetFrame;
                    }
                }
                Fg_stopAcquireEx(currentFg, dmaIndex, memHandle, 0);
                Qylon::log("Finished grabbing in save mode. Acquired frames:" + QString::number(targetFrame+1));
                emit grabbingState(false);
            }
            Fg_FreeMemEx(currentFg, memHandle);
        }).detach();
    }
}
#endif
