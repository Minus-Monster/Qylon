#ifdef VTOOLS_ENABLED
#include "vTools.h"
#include "vTools/vToolsWidget.h"
#include <QDebug>
#include <QImage>
#include <Qylon.h>
#include <QGraphicsItem>

Qylon::vTools::vTools(Qylon *parentQylon) : parent(parentQylon), _waitObject(Pylon::WaitObjectEx::Create())
{
}

Qylon::vTools::~vTools()
{
    delete widget;
}

bool Qylon::vTools::loadRecipe(QString path)
{
    try{
        currentRecipe.DeallocateResources();
        currentRecipe.Unload();

        currentRecipe.Load(path.toStdString().c_str());
        currentRecipe.PreAllocateResources();
        currentRecipe.RegisterAllOutputsObserver(this, Pylon::RegistrationMode_ReplaceAll);

        if(widget !=nullptr) delete widget;
        widget = new vToolsWidget(this);
        widget->loadRecipeConfiguration(&currentRecipe);
    }catch (const Pylon::GenericException &e){
        lastError = e.GetDescription();
        this->parent->log("vTools faced an error " + QString(e.GetDescription()));
        return false;
    }
    return true;
}

void Qylon::vTools::startRecipe()
{
    start();
    Qylon::log("Starting recipe");
}

void Qylon::vTools::stopRecipe()
{
    try{
        requestInterruption();
        _waitObject.Reset();

        currentRecipe.Stop();
        currentRecipe.DeallocateResources();

        QMutexLocker locker(&mutex);
        locker.unlock();
        wait();

        Qylon::log("Recipe stopped");
    }catch(const Pylon::GenericException &e){
        qDebug() << e.what();
    }
}

void Qylon::vTools::run()
{
    if(!currentRecipe.IsLoaded()) return;
    currentRecipe.Start();
    while(!isInterruptionRequested()){
        if(!currentRecipe.IsStarted()){
            return;
        }
        if(getWaitObject().Wait(1000)){
            emit finishedProcessing();
            _waitObject.Reset();
        }
    }
}

void Qylon::vTools::OutputDataPush(Pylon::DataProcessing::CRecipe &recipe, Pylon::DataProcessing::CVariantContainer valueContainer, const Pylon::DataProcessing::CUpdate &update, intptr_t userProvidedId)
{
    PYLON_UNUSED(recipe);
    PYLON_UNUSED(update);
    PYLON_UNUSED(userProvidedId);

    // QPair<Object Name, Object context>
    QList <QPair<QString, Pylon::CPylonImage>> images;
    QList <QPair<QString, QGraphicsItem*>> items;
    QStringList outputTextList;

    try{
        for(const auto &value : valueContainer){
            switch(value.second.GetDataType()){
            case Pylon::DataProcessing::EVariantDataType::VariantDataType_Boolean:{
                if(value.second.IsArray()){
                    for(int i=0; i<value.second.GetNumArrayValues(); ++i){
                        outputTextList.push_back(valueCombinationToString(value.first.c_str(), QString::number(value.second.GetArrayValue(i).ToBool())));
                    }
                }else{
                    outputTextList.push_back(valueCombinationToString(value.first.c_str(), QString::number(value.second.ToBool())));
                }
            } break;
            case Pylon::DataProcessing::VariantDataType_Int64:{
                if(value.second.IsArray()){
                    for(int i=0; i<value.second.GetNumArrayValues(); ++i){
                        outputTextList.push_back(valueCombinationToString(value.first.c_str(), QString::number(value.second.GetArrayValue(i).ToInt64())));
                    }
                }else{
                    outputTextList.push_back(valueCombinationToString(value.first.c_str(), QString::number(value.second.ToInt64())));
                }
            } break;
            case Pylon::DataProcessing::VariantDataType_UInt64:{
                if(value.second.IsArray()){
                    for(int i=0; i<value.second.GetNumArrayValues(); ++i){
                        outputTextList.push_back(valueCombinationToString(value.first.c_str(), QString::number(value.second.GetArrayValue(i).ToUInt64())));
                    }
                }else{
                    outputTextList.push_back(valueCombinationToString(value.first.c_str(), QString::number(value.second.ToUInt64())));
                }
            } break;
            case Pylon::DataProcessing::VariantDataType_String:{
                if(value.second.IsArray()){
                    for(int i=0; i<value.second.GetNumArrayValues(); ++i){
                        outputTextList.push_back(valueCombinationToString(value.first.c_str(), value.second.GetArrayValue(i).ToString().c_str()));
                    }
                }else{
                    outputTextList.push_back(valueCombinationToString(value.first.c_str(), value.second.ToString().c_str()));
                }
            } break;
            case Pylon::DataProcessing::VariantDataType_Float:{
                if(value.second.IsArray()){
                    for(int i=0; i<value.second.GetNumArrayValues(); ++i){
                        outputTextList.push_back(valueCombinationToString(value.first.c_str(), QString::number(value.second.GetArrayValue(i).ToDouble())));
                    }
                }else{
                    outputTextList.push_back(valueCombinationToString(value.first.c_str(), QString::number(value.second.ToDouble())));
                }
            } break;
            case Pylon::DataProcessing::VariantDataType_PylonImage:{
                if(value.second.IsArray()){
                    for(int i=0; i<value.second.GetNumArrayValues(); ++i){
                        QPair<QString, Pylon::CPylonImage> image;
                        image.first = QString(value.first.c_str());
                        image.second = value.second.GetArrayValue(i).ToImage();
                        images.push_back(image);
                    }
                }else{
                    QPair<QString, Pylon::CPylonImage> image;
                    image.first = QString(value.first.c_str());
                    image.second = value.second.ToImage();
                    images.push_back(image);
                }
            } break;
            case Pylon::DataProcessing::VariantDataType_Region:{
                if(value.second.IsArray()){
                    for(int i=0; i<value.second.GetNumArrayValues(); ++i){
                        auto region = value.second.GetArrayValue(i).ToRegion();
                        QRect currentRegion(QPoint(region.GetBoundingBoxTopLeftX(), region.GetBoundingBoxTopLeftY()),
                                            QSize(region.GetBoundingBoxWidth(), region.GetBoundingBoxHeight()));
                        if(region.GetRegionType() == Pylon::DataProcessing::RegionType_RLE32){
                            const Pylon::DataProcessing::SRegionEntryRLE32* regionData = reinterpret_cast<const Pylon::DataProcessing::SRegionEntryRLE32*>(region.GetBufferConst());
                            const size_t entriesCount = region.GetDataSize() / sizeof(Pylon::DataProcessing::SRegionEntryRLE32);

                            for (size_t j = 0; j < entriesCount; ++j){
                                QGraphicsLineItem *lineItem = new QGraphicsLineItem;
                                lineItem->setLine(regionData[j].StartX, regionData[j].Y, regionData[j].EndX, regionData[j].Y);
                                QPair<QString, QGraphicsItem*> item;
                                item.first = QString(value.first.c_str());
                                item.second = lineItem;
                                items.push_back(item);
                            }
                        }else{
                            QGraphicsRectItem *rectItem = new QGraphicsRectItem(currentRegion);
                            QPair<QString, QGraphicsItem*> item;
                            item.first = QString(value.first.c_str());
                            item.second = rectItem;
                            items.push_back(item);
                        }
                        outputTextList.push_back(valueCombinationToString(value.first.c_str(), toString(currentRegion)));
                    }
                }else{
                    auto region = value.second.ToRegion();
                    QRect currentRegion(QPoint(region.GetBoundingBoxTopLeftX(), region.GetBoundingBoxTopLeftY()),
                                        QSize(region.GetBoundingBoxWidth(), region.GetBoundingBoxHeight()));
                    QGraphicsRectItem *rectItem = new QGraphicsRectItem(currentRegion);
                    QPair<QString, QGraphicsItem*> item;
                    item.first = QString(value.first.c_str());
                    item.second = rectItem;
                    items.push_back(item);
                    outputTextList.push_back(valueCombinationToString(value.first.c_str(), toString(currentRegion)));
                }
            } break;
            case Pylon::DataProcessing::VariantDataType_PointF2D:{
                if(value.second.IsArray()){
                    for(int i=0; i<value.second.GetNumArrayValues(); ++i){
                        auto point= value.second.GetArrayValue(i).ToPointF2D();
                        QGraphicsEllipseItem *circleItem = new QGraphicsEllipseItem;
                        double x = point.X-1;
                        double y = point.Y-1;
                        double width = 2;
                        double height = 2;

                        circleItem->setRect(QRectF(x,y,width,height));

                        QPair<QString, QGraphicsItem*> item;
                        item.first = QString(value.first.c_str());
                        item.second = circleItem;
                        items.push_back(item);
                        outputTextList.push_back(valueCombinationToString(value.first.c_str(), toString(QPointF(point.X, point.Y))));
                    }
                }else{
                    auto point= value.second.ToPointF2D();
                    QGraphicsEllipseItem *circleItem = new QGraphicsEllipseItem;
                    double x = point.X-1;
                    double y = point.Y-1;
                    double width = 2;
                    double height = 2;

                    circleItem->setRect(QRectF(x,y,width,height));

                    QPair<QString, QGraphicsItem*> item;
                    item.first = QString(value.first.c_str());
                    item.second = circleItem;
                    items.push_back(item);
                    outputTextList.push_back(valueCombinationToString(value.first.c_str(), toString(QPointF(point.X, point.Y))));
                }
            } break;
            case Pylon::DataProcessing::VariantDataType_LineF2D:{
                if(value.second.IsArray()){
                    for(int i=0; i<value.second.GetNumArrayValues(); ++i){
                        auto line= value.second.GetArrayValue(i).ToLineF2D();
                        QGraphicsLineItem *lineItem = new QGraphicsLineItem;
                        lineItem->setLine(line.PointA.X, line.PointA.Y, line.PointB.X, line.PointB.Y);

                        QPair<QString, QGraphicsItem*> item;
                        item.first = QString(value.first.c_str());
                        item.second = lineItem;
                        items.push_back(item);
                        outputTextList.push_back(valueCombinationToString(value.first.c_str(), toString(lineItem->line())));
                    }
                }else{
                    auto line= value.second.ToLineF2D();
                    QGraphicsLineItem *lineItem = new QGraphicsLineItem;
                    lineItem->setLine(line.PointA.X, line.PointA.Y, line.PointB.X, line.PointB.Y);

                    QPair<QString, QGraphicsItem*> item;
                    item.first = QString(value.first.c_str());
                    item.second = lineItem;
                    items.push_back(item);
                    outputTextList.push_back(valueCombinationToString(value.first.c_str(), toString(lineItem->line())));
                }
            } break;
            case Pylon::DataProcessing::VariantDataType_RectangleF:{
                if(value.second.IsArray()){
                    for(int i=0; i<value.second.GetNumArrayValues(); ++i){
                        auto rect = value.second.GetArrayValue(i).ToRectangleF();
                        QRectF currentRect(rect.Center.X - (rect.Width/2),rect.Center.Y - (rect.Height/2),rect.Width,rect.Height);
                        QGraphicsRectItem *rectItem = new QGraphicsRectItem(currentRect);
                        rectItem->setTransformOriginPoint(currentRect.center());
                        rectItem->setRotation(-((rect.Rotation*180)/3.141592));

                        QPair<QString, QGraphicsItem*> item;
                        item.first = QString(value.first.c_str());
                        item.second = rectItem;
                        items.push_back(item);
                        outputTextList.push_back(valueCombinationToString(value.first.c_str(), toString(currentRect)));

                    }
                }else{
                    auto rect = value.second.ToRectangleF();
                    QRectF currentRect(rect.Center.X - (rect.Width/2),rect.Center.Y - (rect.Height/2),rect.Width,rect.Height);

                    QGraphicsRectItem *rectItem = new QGraphicsRectItem(currentRect);
                    rectItem->setTransformOriginPoint(currentRect.center());
                    rectItem->setRotation(-((rect.Rotation*180)/3.141592));

                    QPair<QString, QGraphicsItem*> item;
                    item.first = QString(value.first.c_str());
                    item.second = rectItem;
                    items.push_back(item);
                    outputTextList.push_back(valueCombinationToString(value.first.c_str(), toString(currentRect)));
                }
            } break;
            case Pylon::DataProcessing::VariantDataType_CircleF:{
                if(value.second.IsArray()){
                    for(int i=0; i<value.second.GetNumArrayValues(); ++i){
                        auto circle= value.second.GetArrayValue(i).ToCircleF();
                        QGraphicsEllipseItem *circleItem = new QGraphicsEllipseItem;
                        double x = circle.Center.X - circle.Radius;
                        double y = circle.Center.Y - circle.Radius;
                        double width = circle.Radius * 2.;
                        double height = circle.Radius * 2.;

                        circleItem->setRect(QRectF(x,y,width,height));

                        QPair<QString, QGraphicsItem*> item;
                        item.first = QString(value.first.c_str());
                        item.second = circleItem;
                        items.push_back(item);
                        outputTextList.push_back(valueCombinationToString(value.first.c_str(), toString(circle)));
                    }
                }else{
                    auto circle= value.second.ToCircleF();
                    QGraphicsEllipseItem *circleItem = new QGraphicsEllipseItem;
                    double x = circle.Center.X - circle.Radius;
                    double y = circle.Center.Y - circle.Radius;
                    double width = circle.Radius * 2.;
                    double height = circle.Radius * 2.;

                    circleItem->setRect(QRectF(x,y,width,height));

                    QPair<QString, QGraphicsItem*> item;
                    item.first = QString(value.first.c_str());
                    item.second = circleItem;
                    items.push_back(item);
                    outputTextList.push_back(valueCombinationToString(value.first.c_str(), toString(circle)));
                }
            } break;
            case Pylon::DataProcessing::VariantDataType_EllipseF:{
                if(value.second.IsArray()){
                    for(int i=0; i<value.second.GetNumArrayValues(); ++i){
                        auto ellipse= value.second.GetArrayValue(i).ToEllipseF();
                        QGraphicsEllipseItem *ellipseItem = new QGraphicsEllipseItem;
                        double x = ellipse.Center.X - ellipse.Radius1;
                        double y = ellipse.Center.Y - ellipse.Radius2;
                        double width = 2.* ellipse.Radius1;
                        double height = 2.* ellipse.Radius2;

                        ellipseItem->setTransformOriginPoint(ellipse.Center.X, ellipse.Center.Y);
                        ellipseItem->setRotation(ellipse.Rotation*180./ 3.141592);
                        ellipseItem->setRect(x,y,width,height);

                        QPair<QString, QGraphicsItem*> item;
                        item.first = QString(value.first.c_str());
                        item.second = ellipseItem;
                        items.push_back(item);
                        outputTextList.push_back(valueCombinationToString(value.first.c_str(), toString(ellipse)));
                    }
                }else{
                    auto ellipse= value.second.ToEllipseF();
                    QGraphicsEllipseItem *ellipseItem = new QGraphicsEllipseItem;
                    double x = ellipse.Center.X - ellipse.Radius1;
                    double y = ellipse.Center.Y - ellipse.Radius2;
                    double width = 2.* ellipse.Radius1;
                    double height = 2.* ellipse.Radius2;
                    ellipseItem->setTransformOriginPoint(ellipse.Center.X, ellipse.Center.Y);
                    ellipseItem->setRotation(ellipse.Rotation*180./ 3.141592);
                    ellipseItem->setRect(x,y,width,height);

                    QPair<QString, QGraphicsItem*> item;
                    item.first = QString(value.first.c_str());
                    item.second = ellipseItem;
                    items.push_back(item);
                    outputTextList.push_back(valueCombinationToString(value.first.c_str(), toString(ellipse)));
                }

            } break;
            case Pylon::DataProcessing::VariantDataType_TransformationData:{qDebug() << "Transformation";} break;
            case Pylon::DataProcessing::VariantDataType_Composite:{ qDebug() << "Composite";} break;
            case Pylon::DataProcessing::VariantDataType_None:{ qDebug() << "None";} break;
            case Pylon::DataProcessing::VariantDataType_Generic:
            case Pylon::DataProcessing::VariantDataType_Unsupported:
                break;
            }
        }
        Pylon::AutoLock scopedLock(_memberLock);
        Result result;
        result.images = images;
        result.items = items;
        result.strings = outputTextList;

        resultsQueue.push_back(result);
        _waitObject.Signal();
    }catch(const Pylon::GenericException &e){
        qDebug() << e.GetDescription();
    }
}

QImage Qylon::vTools::getSelectedImage(QList<QPair<QString, Pylon::CPylonImage> > images){
    Pylon::CPylonImage selected;
    for(int i=0; i<images.size(); ++i){
        if(images.at(i).first == filter->currentImage){
            selected = images.at(i).second;
            break;
        }
    }
    return convertPylonImageToQImage(selected);
}

QString Qylon::vTools::getParseredString(QStringList strings){
    QString outputText;
    for(int i=0; i<filter->items.size(); ++i){
        if(!filter->items.at(i).second){
            for(int j=0; j<strings.size(); ++j){
                if(strings.at(j).contains(filter->items.at(i).first)){
                    strings.removeAll(strings.at(j));
                }
            }
        }
    }
    if(strings.isEmpty()) outputText = "";
    else outputText = strings.join(filter->itemDelimiter) + filter->itemDelimiter;
    return outputText;
}

QWidget *Qylon::vTools::getWidget()
{
    return widget;
}

QString Qylon::vTools::valueCombinationToString(QString object, QString value)
{
    return object + filter->keyValueDelimiter + value;
}

QString Qylon::vTools::toString(QLineF line)
{
    return "P1XY-P2XY[" + QString::number(line.p1().x()) + "," + QString::number(line.p1().y()) + "," + QString::number(line.p2().x()) + "," + QString::number(line.p2().y()) + "]";
}

QString Qylon::vTools::toString(QRectF rect)
{
    return "XYWH[" + QString::number(rect.x()) + "," + QString::number(rect.y()) + "," + QString::number(rect.width()) + "," + QString::number(rect.height()) + "]";
}

QString Qylon::vTools::toString(QPointF point)
{
    return "XY[" + QString::number(point.x()) + "," + QString::number(point.y()) +"]";
}

QString Qylon::vTools::toString(Pylon::DataProcessing::SEllipseF ellipse)
{
    return "CenterXY-Rotation-XRadius-YRadius[" + QString::number(ellipse.Center.X) + "," + QString::number(ellipse.Center.Y) +"," + QString::number(ellipse.Rotation) + "," + QString::number(ellipse.Radius1) + "," + QString::number(ellipse.Radius2)+"]";
}

QString Qylon::vTools::toString(Pylon::DataProcessing::SCircleF circle)
{
    return "CenterXY-Radius[" + QString::number(circle.Center.X) + "," + QString::number(circle.Center.Y) + "," + QString::number(circle.Radius) +"]";
}

#endif

