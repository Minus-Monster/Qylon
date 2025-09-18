#ifndef VTOOLS_H
#define VTOOLS_H
#ifdef VTOOLS_ENABLED
#include <QImage>
#include <QObject>
#include <QThread>
#include <QMutexLocker>
#include <QGraphicsItem>
#include <QComboBox>
#include <QCheckBox>
#include <QWaitCondition>
#include <pylondataprocessing/PylonDataProcessingIncludes.h>

namespace Qylon{
class Qylon;
class vToolsWidget;
class vTools : public QThread, public Pylon::DataProcessing::IOutputObserver
{
    Q_OBJECT
public:
    struct ResultFilter{
        QString currentImage;
        QList<QPair<QString, bool>> items;
        QString itemDelimiter =";";
        QString keyValueDelimiter = "=";
    };
    struct Result{
        QList<QPair<QString, Pylon::CPylonImage>> images;
        QList<QPair<QString, QGraphicsItem*>> items;
        QStringList strings;
    };
    vTools(Qylon *parentQylon=nullptr);
    ~vTools();
    bool loadRecipe(QString path);
    void startRecipe(int count=-1);
    void stopRecipe();
    QString getCurrentRecipePath(){ return currentRecipePath; }
    Pylon::DataProcessing::CRecipe *getRecipe(){ return &currentRecipe; }
    void run() override;

    void OutputDataPush(Pylon::DataProcessing::CRecipe &recipe,
                        Pylon::DataProcessing::CVariantContainer valueContainer,
                        const Pylon::DataProcessing::CUpdate &update,
                        intptr_t userProvidedId) override;
    Qylon *getQylon(){ return parent; }
    QImage getSelectedImage(QList<QPair<QString, Pylon::CPylonImage>> images);
    QString getParseredString(QStringList strings);
    const Pylon::WaitObject& getWaitObject(){ return _waitObject; }
    void setResultFilter(ResultFilter *resultFilter){ filter = resultFilter;}
    QWidget *getWidget();
    QString valueCombinationToString(QString object, QString value);
    QString toString(QLineF line);
    QString toString(QRectF rect);
    QString toString(QPointF point);
    QString toString(Pylon::DataProcessing::SEllipseF ellipse);
    QString toString(Pylon::DataProcessing::SCircleF circle);
    QString getLastError(){ return lastError; }
    Result getResult();

    Result getContainerResult(Pylon::DataProcessing::CVariantContainer container);

signals:
    void finishedProcessing();

private:
    Qylon *parent = nullptr;
    QMutex mutex;
    QString lastError = "";
    int counter = 0;
    QString currentRecipePath;

    Pylon::DataProcessing::CRecipe currentRecipe;
    Pylon::CLock _memberLock;
    Pylon::WaitObjectEx _waitObject;

    QList<Result> resultsQueue;
    ResultFilter *filter;
    vToolsWidget *widget=nullptr;
};
}

#endif // VTOOLS_H
#endif
