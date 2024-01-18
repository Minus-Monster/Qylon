#include "vTools.h"
#include "qpainter.h"
#include <QDebug>
#include <QImage>
#include <Qylon.h>

Qylon::vTools::vTools(Qylon *parentQylon) : parent(parentQylon), _waitObject(Pylon::WaitObjectEx::Create())
{
}

void Qylon::vTools::loadRecipe(QString path)
{
    try{
        currentRecipe.DeallocateResources();
        currentRecipe.Unload();

        currentRecipe.Load(path.toStdString().c_str());
        currentRecipe.PreAllocateResources();
        currentRecipe.RegisterAllOutputsObserver(this, Pylon::RegistrationMode_ReplaceAll);

    }catch (const GenICam_3_1_Basler_pylon::GenericException &e){
        this->parent->log("vTools faced an error" + QString(e.GetDescription()));
    }
}

void Qylon::vTools::startRecipe()
{
    currentRecipe.Start();
}

void Qylon::vTools::stopRecipe()
{
    currentRecipe.Stop();
}

void Qylon::vTools::OutputDataPush(Pylon::DataProcessing::CRecipe &recipe, Pylon::DataProcessing::CVariantContainer valueContainer, const Pylon::DataProcessing::CUpdate &update, intptr_t userProvidedId)
{
    PYLON_UNUSED(recipe);
    PYLON_UNUSED(update);
    PYLON_UNUSED(userProvidedId);

    Pylon::CPylonImage pImage;
    QList<QRect> boundaries;
    QList<QPair<QRectF, double>> rectangles;
    QList<QPointF> points;
    try{
        for(const auto value : valueContainer){
            qDebug() << "VALUE NAME : " << value.first;
            switch(value.second.GetDataType()){
            case Pylon::DataProcessing::EVariantDataType::VariantDataType_Boolean:{ qDebug() << "Boolean";} break;
            case Pylon::DataProcessing::VariantDataType_Int64:{ qDebug() << "Int";} break;
            case Pylon::DataProcessing::VariantDataType_UInt64:{ qDebug() << "UInt";} break;
            case Pylon::DataProcessing::VariantDataType_String:{ qDebug() << "String";} break;
            case Pylon::DataProcessing::VariantDataType_Float:{ qDebug() << "Float";} break;
            case Pylon::DataProcessing::VariantDataType_PylonImage:{
                qDebug() << "Image";
                pImage = value.second.ToImage();
            } break;
            case Pylon::DataProcessing::VariantDataType_Region:{
                qDebug() << "Region";
                if(value.second.IsArray()){
                    for(int i=0; i<value.second.GetNumArrayValues(); ++i){
                        auto region = value.second.GetArrayValue(i).ToRegion();
                        QRect currentRegion(QPoint(region.GetBoundingBoxTopLeftX(), region.GetBoundingBoxTopLeftY()),
                                            QSize(region.GetBoundingBoxWidth(), region.GetBoundingBoxHeight()));
                        boundaries.push_back(currentRegion);
                    }
                }else{
                    auto region = value.second.ToRegion();
                    QRect currentRegion(QPoint(region.GetBoundingBoxTopLeftX(), region.GetBoundingBoxTopLeftY()),
                                        QSize(region.GetBoundingBoxWidth(), region.GetBoundingBoxHeight()));
                    boundaries.push_back(currentRegion);
                }
            } break;
            case Pylon::DataProcessing::VariantDataType_TransformationData:{qDebug() << "Transformation";} break;
            case Pylon::DataProcessing::VariantDataType_Composite:{ qDebug() << "Composite";} break;
            case Pylon::DataProcessing::VariantDataType_PointF2D:{
                qDebug() << "PointF2D";
                if(value.second.IsArray()){
                    for(int i=0; i<value.second.GetNumArrayValues(); ++i){
                        auto point = value.second.GetArrayValue(i).ToPointF2D();
                        QPointF currentPoint(point.X, point.Y);
                        points.push_back(currentPoint);
                    }
                }else{
                    auto point = value.second.ToPointF2D();
                    QPointF currentPoint(point.X, point.Y);
                    points.push_back(currentPoint);
                }
            } break;
            case Pylon::DataProcessing::VariantDataType_LineF2D:{qDebug() << "LineF2D";} break;
            case Pylon::DataProcessing::VariantDataType_RectangleF:{
                qDebug() << "RectangleF";
                if(value.second.IsArray()){
                    for(int i=0; i<value.second.GetNumArrayValues(); ++i){
                        auto rect = value.second.GetArrayValue(i).ToRectangleF();
                        QPair<QRectF, double> currentRect;
                        currentRect.first.setRect(rect.Center.X - (rect.Width/2),rect.Center.Y - (rect.Height/2),rect.Width,rect.Height);
                        currentRect.second = -((rect.Rotation*180)/3.141592);
                        rectangles.push_back(currentRect);
                    }
                }else{
                    auto rect = value.second.ToRectangleF();
                    QPair<QRectF, double> currentRect;
                    currentRect.first.setRect(rect.Center.X - (rect.Width/2),rect.Center.Y - (rect.Height/2),rect.Width,rect.Height);
                    currentRect.second = -((rect.Rotation*180)/3.141592);
                    rectangles.push_back(currentRect);
                }
            } break;
            case Pylon::DataProcessing::VariantDataType_CircleF:{qDebug() << "CircleF";} break;
            case Pylon::DataProcessing::VariantDataType_EllipseF:{qDebug() << "EllipseF";} break;
            case Pylon::DataProcessing::VariantDataType_None:{ qDebug() << "None";} break;
            }
        }
        if(pImage.IsValid()){
            qDebug() << "Is BGR?" << Pylon::IsBGR(pImage.GetPixelType());
            if(Pylon::IsBGR(pImage.GetPixelType())){
                outputImage = QImage(pImage.GetWidth(), pImage.GetHeight(), QImage::Format_RGB32);

                Pylon::CImageFormatConverter converter;
                converter.OutputPixelFormat = Pylon::PixelType_BGRA8packed;
                converter.Convert(outputImage.bits(), outputImage.sizeInBytes(), pImage);
            }else{
                outputImage = QImage((uchar*)pImage.GetBuffer(),pImage.GetWidth(), pImage.GetHeight(), QImage::Format_RGB32);
            }
            QPainter painter(&outputImage);
            painter.setBrush(Qt::NoBrush);

            if(!boundaries.empty()){
                int cntRegion = 0;
                for(auto current : boundaries){
                    painter.setPen(QColor(249,157,51,255));
                    painter.setBrush(Qt::NoBrush);

                    painter.drawText(current.topLeft(), "Region " + QString::number(++cntRegion));
                    painter.drawRect(current);
                }
            }
            if(!rectangles.empty()){
                int cntRectangle = 0;
                for(auto current : rectangles){
                    painter.save();
                    painter.translate(current.first.center());
                    painter.rotate(current.second);
                    QRectF transformed(-(current.first.width()*0.5), -(current.first.height()*0.5),
                                       current.first.width(), current.first.height());

                    painter.setPen(QColor(20,72,126,255));
                    painter.drawText(transformed.topLeft(), "Rectangle " + QString::number(++cntRectangle));
                    painter.setPen(Qt::NoPen);
                    painter.setBrush(QBrush(QColor(20,72,126,200), Qt::SolidPattern));
                    painter.drawRect(transformed);
                    painter.setBrush(Qt::NoBrush);
                    painter.restore();
                }
            }
            if(!points.empty()){
                int cntPoints = 0;
                for(auto current : points){

                    painter.setPen(QColor(0,255,0));
                    painter.drawPoint(current);
                }
            }
            painter.end();
        }else{
            qDebug() << "Image is not valid";
        }
        emit finished();
    }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
        qDebug() << e.GetDescription();
    }

}

Qylon::Qylon *Qylon::vTools::getQylon()
{
    return parent;
}
