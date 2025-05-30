#ifndef GRABBER_H
#define GRABBER_H
#ifdef GRABBER_ENABLED
#include "basler_fg.h"
#include "fg_struct.h"
#include "sisoIo.h"
#undef Sleep // define to avoid the conflict with the Sleep macro in os_funcs.h

#include <QObject>
#include <QImage>
#include <QDebug>
#include <QThread>
#include <QFutureWatcher>

#include <map>
#include <atomic>
#include <mutex>

namespace Qylon{
class GrabberWidget;
class Qylon;
class Grabber : public QThread
{
    Q_OBJECT
public:
    struct APC {
        Grabber *address;
        unsigned int dmaIndex;
        QList<unsigned short*> imageBuffer;
        dma_mem *memBuf;
        int width;
        int height;
        bool allocated=false;
    };

    Grabber(Qylon *parentQylon = nullptr, unsigned int boardIndex=0);
    ~Grabber();
    Qylon* getQylon(){ return parent; }
    static int CallbackFromGrabber(frameindex_t picNr, void *ctx);
    bool loadApplet(QString file);
    bool loadConfiguration(QString file, bool ignoringError=false);

    int getDMACount();
    int getWidth(int dmaIndex);
    int getHeight(int dmaIndex);
    int getX(int dmaIndex);
    int getY(int dmaIndex);
    int getBytesPerPixel(int dmaIndex=0);

    void setTriggerMode(MeTriggerMode mode){
        setParameterValue(FG_TRIGGERMODE, mode);
    }

    FgParamTypes getParameterDataType(QString typeName, int dmaIndex);
    QString getParameterStringValue(QString typeName, int dmaIndex);
    int getParameterIntValue(QString typeName, int dmaIndex);
    double getParameterDoubleValue(QString typeName, int dmaIndex);

    Fg_Struct* getFg();
    APC *getAPC(int dmaIndex);
    int getLastFrameCount(int dmaIndex);

    void setParameterValue(QString typeName, int value, int dmaIndex=0);
    void setParameterValue(QString typeName, double value, int dmaIndex=0);
    void setParameterValue(QString typeName, QString value, int dmaIndex=0);
    void setParameterValue(int typeNum, int value, int dmaIndex=0);

    bool registerAPCHandler(int imgBufCnt=1);

    QWidget *getWidget();
    void release();
    void allocateImageBuffer(int dmaIndex);
    void deallocateImageBuffer(int dmaIndex);
    bool saveCurrentConfig(QString fileName);
    bool isInitialized(){ return currentFg != nullptr; }

signals:
    void sendImage(const QImage &image, unsigned int dmaIdx=0);
    void grabbed();
    void loadedApplet(QString appletPath);
    void loadedConfig(QString configPath);
    void connectionStatus(bool connected);
    void grabbingState(bool isGrabbing);
    void updatedParametersValue();
    void released();

public slots:
    void singleGrab(int dmaIndex=0);
    void continuousGrab(int dmaIndex=0);
    void sequentialGrab(int numFrame, int dmaIndex=0);
    void stopGrab(int dmaIndex=0);
    void stopGrabAll();
    void grabThreadLoop(int numFrame=0, int dmaIndex=0);
    void stopThreadLoop(int dmaIndex=0);
    void saveImage(QString dir, QString fileName, int numFrame=1, int dmaIndex=0);
    void setImage(const QImage image, int dmaIndex=1);
    void setBuffer(unsigned short* buffer, int width, int height, int depth, int dmaIndex=1);

private:
    Qylon *parent = nullptr;
    Fg_Struct *currentFg = nullptr;
    QList<APC*> apcDataList;
    int boardNumIndex =0;           // physical number of grabber in PC.
    int imagePushCnt=0;             // When if use two or more image buffers to put images into grabber, it would be the counter.
    int imageBufferSize=1;          // the buffer size putting images into grabber.

    std::mutex grabberMutex;

    // For the threading mode
    dma_mem *memHandle=nullptr;
    std::map<int, std::atomic<bool>> stopFlags;

    GrabberWidget *widget;

protected:
    // void run() override;
};}
#endif
#endif // GRABBER_H
