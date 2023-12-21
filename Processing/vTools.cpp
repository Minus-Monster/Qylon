#include "vTools.h"
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

}

void Qylon::vTools::OutputDataPush(Pylon::DataProcessing::CRecipe &recipe, Pylon::DataProcessing::CVariantContainer valueContainer, const Pylon::DataProcessing::CUpdate &update, intptr_t userProvidedId)
{
    PYLON_UNUSED(recipe);
    PYLON_UNUSED(update);
    PYLON_UNUSED(userProvidedId);

    Pylon::CPylonImage pImage;
    try{
        for(const auto value : valueContainer){
            qDebug() << value.first;
            switch(value.second.GetDataType()){
            case Pylon::DataProcessing::EVariantDataType::VariantDataType_Boolean:{} break;
            case Pylon::DataProcessing::VariantDataType_Int64:{} break;
            case Pylon::DataProcessing::VariantDataType_UInt64:{} break;
            case Pylon::DataProcessing::VariantDataType_String:{} break;
            case Pylon::DataProcessing::VariantDataType_Float:{} break;
            case Pylon::DataProcessing::VariantDataType_PylonImage:{
                pImage = value.second.ToImage();
            } break;
            case Pylon::DataProcessing::VariantDataType_Region:{
                if(value.second.IsArray()){

                    qDebug() << "Array size" << value.second.GetNumArrayValues();
                    for(int i=0; i<value.second.GetNumArrayValues(); ++i){
                        qDebug() << i ;
                        value.second.GetArrayValue(i).ToRegion().AttachUserBuffer(pImage.GetBuffer(),pImage.GetImageSize(), pImage.GetImageSize(), Pylon::DataProcessing::ERegionType::RegionType_RLE32);
                    }
                }
            } break;
            case Pylon::DataProcessing::VariantDataType_TransformationData:{} break;
            case Pylon::DataProcessing::VariantDataType_Composite:{} break;
            case Pylon::DataProcessing::VariantDataType_PointF2D:{} break;
            case Pylon::DataProcessing::VariantDataType_LineF2D:{} break;
            case Pylon::DataProcessing::VariantDataType_RectangleF:{} break;
            case Pylon::DataProcessing::VariantDataType_CircleF:{} break;
            case Pylon::DataProcessing::VariantDataType_EllipseF:{} break;
            case Pylon::DataProcessing::VariantDataType_None:{} break;
            }
        }
        outputImage = QImage((uchar*)pImage.GetBuffer(),pImage.GetWidth(), pImage.GetHeight(), QImage::Format_RGB32);
        emit finished();
    }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
        qDebug() << e.GetDescription();
    }

}

Qylon::Qylon *Qylon::vTools::getQylon()
{
    return parent;
}
