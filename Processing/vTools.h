#ifndef VTOOLS_H
#define VTOOLS_H

#include <QImage>
#include <QObject>
#include <QThread>
#include <QMutexLocker>
#include <QGraphicsItem>
#include <pylondataprocessing/PylonDataProcessingIncludes.h>

namespace Qylon{
class Qylon;
class vTools : public QThread, public Pylon::DataProcessing::IOutputObserver
{
    Q_OBJECT
public:
    vTools(Qylon *parentQylron=nullptr);
    bool loadRecipe(QString path);
    void startRecipe();
    void stopRecipe();
    void run() override;

    void OutputDataPush(Pylon::DataProcessing::CRecipe &recipe,
                        Pylon::DataProcessing::CVariantContainer valueContainer,
                        const Pylon::DataProcessing::CUpdate &update,
                        intptr_t userProvidedId) override;
    Qylon *getQylon(){
        return parent;
    }
    const QImage &getImage(){
        return outputImage;
    }
    const QString &getOutputText(){
        return outputText;
    }
    const Pylon::WaitObject& getWaitObject(){
        return _waitObject;
    }
    const QList<QPair<QString, Pylon::CPylonImage>> getImages(){
        return images;
    }
    const QList<QPair<QString, QGraphicsItem*>> getItems(){
        return items;
    }
    const QList<QPair<QString, QString>> getStrings(){
        return strings;
    }

signals:
    void finished();

private:
    Qylon *parent = nullptr;
    Pylon::DataProcessing::CRecipe currentRecipe;
    GenApi_3_1_Basler_pylon::CLock _memberLock;
    Pylon::WaitObjectEx _waitObject;

    QImage outputImage;
    QString outputText;

    QList<QPair<QString, Pylon::CPylonImage>> images;
    QList<QPair<QString, QGraphicsItem*>> items;
    QList<QPair<QString, QString>> strings;
};
}

#endif // VTOOLS_H
