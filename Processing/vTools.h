#ifndef VTOOLS_H
#define VTOOLS_H

#include "qimage.h"
#include <QObject>
#include <pylondataprocessing/PylonDataProcessingIncludes.h>

namespace Qylon{
class Qylon;
class vTools : public QObject, public Pylon::DataProcessing::IOutputObserver
{
    Q_OBJECT
public:
    struct SizeInfo{
        double width=-1.;
        double height=-1.;
        double rad=-1.;
        struct Center{
            double x=-1.;
            double y=-1.;
        };
        Center center;
    };
    struct Composite{
        QImage image;
        QList<QString> stringValues;
        QList<SizeInfo> boundaries;
        QList<bool> booleans;
        QList<double> doubles;
        QList<float> floats;
        QList<int> integers;
        QList<unsigned int> uIntegers;
    };
    vTools(Qylon *parentQylon=nullptr);
    void loadRecipe(QString path);
    void startRecipe();
    void stopRecipe();

    void OutputDataPush(Pylon::DataProcessing::CRecipe &recipe,
                        Pylon::DataProcessing::CVariantContainer valueContainer,
                        const Pylon::DataProcessing::CUpdate &update,
                        intptr_t userProvidedId) override;
    Qylon *getQylon();

    const QImage &getImage(){ return outputImage; }

signals:
    void finished();


private:
    Qylon *parent = nullptr;
    Pylon::DataProcessing::CRecipe currentRecipe;
    GenApi_3_1_Basler_pylon::CLock _memberLock;
    Pylon::WaitObjectEx _waitObject;

    QImage outputImage;
};
}

#endif // VTOOLS_H
