#ifndef IMAGETOOLS_H
#define IMAGETOOLS_H
#include <QImage>
#include <QDebug>
#ifdef PYLON_ENABLED
#include <pylon/ImageFormatConverter.h>
#include <pylon/PylonImage.h>
#endif
namespace Qylon{
/// <summary>
/// Only effects for 16bit grayscale image.
/// shift value of 8 is equal to Higher 8bit image.
/// </summary>
inline QImage shiftImage(QImage original, int shift=8){
    int width = original.width();
    int height = original.height();
    QImage output = QImage(width, height, QImage::Format_Grayscale8);

    for(int y=0; y < height; ++y){
        const uint16_t* srcLine = reinterpret_cast<const uint16_t*>(original.scanLine(y));
        uint8_t* outputByteLine = output.scanLine(y);
        for (int x = 0; x < width; ++x) {
            uint16_t pixelValue = srcLine[x];
            // shift = 8 is equal to Higher 8bits image.
            outputByteLine[x] = static_cast<uint8_t>((pixelValue >> shift) & 0xFF);
        }
    }
    return output;
}
#ifdef PYLON_ENABLED
inline QImage convertPylonImageToQImage(Pylon::CPylonImage pylonImg){
    Pylon::CImageFormatConverter converter;
    QImage::Format format = QImage::Format_Invalid;
    switch(pylonImg.GetPixelType()){
    case Pylon::PixelType_Undefined:
    case Pylon::PixelType_Mono1packed:
    case Pylon::PixelType_Mono2packed:
    case Pylon::PixelType_Mono4packed:
    case Pylon::PixelType_Mono8:
    case Pylon::PixelType_Mono8signed:
        format = QImage::Format_Grayscale8;
        break;
    case Pylon::PixelType_Mono10:
    case Pylon::PixelType_Mono10packed:
    case Pylon::PixelType_Mono10p:
    case Pylon::PixelType_Mono12:
    case Pylon::PixelType_Mono12packed:
    case Pylon::PixelType_Mono12p:
    case Pylon::PixelType_Mono16:
        format = QImage::Format_Grayscale16;
        break;
    case Pylon::PixelType_BayerGR8:
    case Pylon::PixelType_BayerRG8:
    case Pylon::PixelType_BayerGB8:
    case Pylon::PixelType_BayerBG8:
    case Pylon::PixelType_BayerGR10:
    case Pylon::PixelType_BayerRG10:
    case Pylon::PixelType_BayerGB10:
    case Pylon::PixelType_BayerBG10:
    case Pylon::PixelType_BayerGR12:
    case Pylon::PixelType_BayerRG12:
    case Pylon::PixelType_BayerGB12:
    case Pylon::PixelType_BayerBG12:
    case Pylon::PixelType_RGB8packed:
    case Pylon::PixelType_BGR8packed:
    case Pylon::PixelType_RGBA8packed:
    case Pylon::PixelType_BGRA8packed:
    case Pylon::PixelType_RGB10packed:
    case Pylon::PixelType_BGR10packed:
    case Pylon::PixelType_RGB12packed:
    case Pylon::PixelType_BGR12packed:
    case Pylon::PixelType_RGB16packed:
    case Pylon::PixelType_BGR10V1packed:
    case Pylon::PixelType_BGR10V2packed:
    case Pylon::PixelType_YUV411packed:
    case Pylon::PixelType_YUV422packed:
    case Pylon::PixelType_YUV444packed:
    case Pylon::PixelType_RGB8planar:
    case Pylon::PixelType_RGB10planar:
    case Pylon::PixelType_RGB12planar:
    case Pylon::PixelType_RGB16planar:
    case Pylon::PixelType_YUV422_YUYV_Packed:
    case Pylon::PixelType_YUV444planar:
    case Pylon::PixelType_YUV422planar:
    case Pylon::PixelType_YUV420planar:
    case Pylon::PixelType_YCbCr420_8_YY_CbCr_Semiplanar:
    case Pylon::PixelType_YCbCr422_8_YY_CbCr_Semiplanar:
    case Pylon::PixelType_BayerGR12Packed:
    case Pylon::PixelType_BayerRG12Packed:
    case Pylon::PixelType_BayerGB12Packed:
    case Pylon::PixelType_BayerBG12Packed:
    case Pylon::PixelType_BayerGR10p:
    case Pylon::PixelType_BayerRG10p:
    case Pylon::PixelType_BayerGB10p:
    case Pylon::PixelType_BayerBG10p:
    case Pylon::PixelType_BayerGR12p:
    case Pylon::PixelType_BayerRG12p:
    case Pylon::PixelType_BayerGB12p:
    case Pylon::PixelType_BayerBG12p:
    case Pylon::PixelType_BayerGR16:
    case Pylon::PixelType_BayerRG16:
    case Pylon::PixelType_BayerGB16:
    case Pylon::PixelType_BayerBG16:
    case Pylon::PixelType_RGB12V1packed:
        format = QImage::Format_RGB32;
        break;
    case Pylon::PixelType_Double:
    case Pylon::PixelType_Confidence8:
    case Pylon::PixelType_Confidence16:
    case Pylon::PixelType_Coord3D_C8:
    case Pylon::PixelType_Coord3D_C16:
    case Pylon::PixelType_Coord3D_ABC32f:
    case Pylon::PixelType_Data8:
    case Pylon::PixelType_Data8s:
    case Pylon::PixelType_Data16:
    case Pylon::PixelType_Data16s:
    case Pylon::PixelType_Data32:
    case Pylon::PixelType_Data32s:
    case Pylon::PixelType_Data64:
    case Pylon::PixelType_Data64s:
    case Pylon::PixelType_Data32f:
    case Pylon::PixelType_Data64f:
        break;
    }
    QImage outImage(pylonImg.GetWidth(), pylonImg.GetHeight(), format);
    if(format != QImage::Format_RGB32){
        // converter.OutputPixelFormat = pylonImg.GetPixelType();
        // converter.Convert(outImage.bits(), (size_t)pylonImg.GetImageSize(), pylonImg);
        // memcpy(outImage.bits(), pylonImg.GetBuffer(), pylonImg.GetImageSize());
        int width = pylonImg.GetWidth();
        int height = pylonImg.GetHeight();
        const uchar* buffer = static_cast<const uchar*>(pylonImg.GetBuffer());
        outImage = QImage(buffer, width, height, width, format).copy();
    }else{
        converter.OutputPixelFormat = Pylon::PixelType_BGRA8packed;
        converter.Convert(outImage.bits(), outImage.sizeInBytes(), pylonImg);
    }
    return outImage;
}



#endif
}
#endif // IMAGETOOLS_H
