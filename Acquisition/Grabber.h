#ifndef GRABBER_H
#define GRABBER_H
#ifdef GRABBER_ENABLED
#include "basler_fg.h"
#include "fg_struct.h"

#include <QObject>
#include <QImage>
#include <QDebug>

#define BUF_NUM 4

namespace Qylon{
class Grabber;
struct APC {
    Grabber *address;
    unsigned int dmaIndex;
    unsigned short* imageBuffer;
    dma_mem *memBuf;
    int width;
    int height;
};

class GrabberWidget;
class Qylon;
class Grabber : public QObject
{
    Q_OBJECT
public:
    Grabber(Qylon *parentQylon = nullptr, unsigned int boardIndex=0);
    bool loadApplet(QString file);
    bool loadConfiguration(QString file);
    int getWidth(int dmaIndex);
    int getHeight(int dmaIndex);
    int getPixelDepth(int dmaIndex);
    void setParameterValue(QString typeName, int value, int dmaIndex=0);
    int getParameterValue(QString typeName, int dmaIndex=0);
    void initialize(int dmaCount=1);
    void reitialize(int dmaCount=1);
    int getBytesPerPixel(int dmaIndex=0);

    static int CallbackFromGrabber(frameindex_t picNr, void *ctx);
    Fg_Struct* getFg();
    APC *getAPC(int dmaIndex);
    QWidget *getWidget();
    int getCurrentDMACount(){
        return currentDmaCount;
    }

    void release();

signals:
    void sendImage(const QImage &image, unsigned int dmaIdx=0);

public slots:
    void singleGrab(int dmaIndex=0);
    void continuousGrab(int dmaIndex=0);
    void sequentialGrab(int numFrame, int dmaIndex=0);
    void stopGrab(int dmaIndex=0);

private:
    Qylon *parent = nullptr;
    Fg_Struct *currentFg = nullptr;
    QList<APC*> apcDataList;
    int currentDmaCount=1;
    int boardNumIndex =0;

    GrabberWidget *widget;
};}
#endif
#endif // GRABBER_H
