#ifndef GRABBER_H
#define GRABBER_H
#ifdef GRABBER_ENABLED
#include "basler_fg.h"
#include "fg_struct.h"

#include <QObject>
#include <QImage>
#include <QDebug>

namespace Qylon{
class GrabberWidget;
class Qylon;
class Grabber : public QObject
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
    };
    Grabber(Qylon *parentQylon = nullptr, unsigned int boardIndex=0);
    bool loadApplet(QString file);
    bool loadConfiguration(QString file);
    int getDMACount();
    int getWidth(int dmaIndex);
    int getHeight(int dmaIndex);
    int getX(int dmaIndex);
    int getY(int dmaIndex);
    int getPixelDepth(int dmaIndex);
    void setParameterValue(QString typeName, int value, int dmaIndex=0);
    void setParameterValue(int typeNum, int value, int dmaIndex=0);
    int getParameterValue(QString typeName, int dmaIndex=0);
    bool initialize(int imgBufCnt=1);
    void reitialize(int imgBufCnt=1);
    int getBytesPerPixel(int dmaIndex=0);
    void setImage(const QImage image);

    static int CallbackFromGrabber(frameindex_t picNr, void *ctx);
    Fg_Struct* getFg();
    APC *getAPC(int dmaIndex);
    QWidget *getWidget();
    void release();

signals:
    void sendImage(const QImage &image, unsigned int dmaIdx=0);

public slots:
    void singleGrab(int dmaIndex=0);
    void continuousGrab(int dmaIndex=0);
    void sequentialGrab(int numFrame, int dmaIndex=0);
    void stopGrab(int dmaIndex=0);
    void stopGrabAll();

private:
    Qylon *parent = nullptr;
    Fg_Struct *currentFg = nullptr;
    QList<APC*> apcDataList;
    int boardNumIndex =0;           // physical number of grabber in PC.
    int imagePushCnt=0;             // When if use two or more image buffers to put images into grabber, it would be the counter.
    int imageBufferSize=1;          // the buffer size putting images into grabber.

    GrabberWidget *widget;
};}
#endif
#endif // GRABBER_H
