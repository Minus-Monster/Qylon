#ifndef TIFFREADER_H
#define TIFFREADER_H
#include <QImage>
#include <QDebug>
#include "tiffio.h"

namespace Qylon{
inline const QImage openTiff(QString filePath){
    qDebug() << filePath;
    TIFF *tiff = TIFFOpen(filePath.toUtf8(), "r");
    if(!tiff){
        qDebug() << "Failed to open TIFF";
        return QImage();
    }
    // Get image properties
    uint32_t width, height;
    TIFFGetField(tiff, TIFFTAG_IMAGEWIDTH, &width);
    TIFFGetField(tiff, TIFFTAG_IMAGELENGTH, &height);
    uint32_t imageSize = width * height;

    // Get photometric interpretation (color mode)
    uint16_t photometric;
    TIFFGetField(tiff, TIFFTAG_PHOTOMETRIC, &photometric);

    // Allocate memory for image buffer
    uint32_t *buffer = new uint32_t[imageSize];
    if (!buffer) {
        qWarning() << "Failed to allocate memory for TIFF image";
        TIFFClose(tiff);
        return QImage();
    }

    // Read TIFF image data
    bool success = false;
    switch (photometric) {
    case PHOTOMETRIC_RGB:
        success = TIFFReadRGBAImage(tiff, width, height, buffer, 0);
        break;
    case PHOTOMETRIC_PALETTE:
    case PHOTOMETRIC_MINISBLACK:
        success = TIFFReadEncodedStrip(tiff, 0, buffer, imageSize * sizeof(uint32_t)) != -1;
        break;
        // Handle other photometric interpretations as needed
    }

    if (!success) {
        qWarning() << "Failed to read TIFF image data";
        delete[] buffer;
        TIFFClose(tiff);
        return QImage();
    }
    uint32_t bitsPerSample = 0;
    TIFFGetField(tiff, TIFFTAG_BITSPERSAMPLE, &bitsPerSample);
    // Convert TIFF image data to QImage with correct format
    QImage::Format imageFormat = QImage::Format_Invalid;
    switch (photometric) {
    case PHOTOMETRIC_RGB:
        imageFormat = (bitsPerSample==16) ? QImage::Format_RGBX64 : QImage::Format_RGBA8888;
        break;
    case PHOTOMETRIC_PALETTE:
        imageFormat = QImage::Format_Indexed8;
        break;
    case PHOTOMETRIC_MINISBLACK:
        qDebug() << bitsPerSample;
        imageFormat = (bitsPerSample==16) ? QImage::Format_Grayscale16 : QImage::Format_Grayscale8;
        break;
        // Handle other photometric interpretations as needed
    }

    QImage image(width, height, imageFormat);
    //    for (int y = 0; y < height; ++y) {
    //        for (int x = 0; x < width; ++x) {
    //            uint32_t color = buffer[(height - y - 1) * width + x]; // 높이에서 y를 빼서 위아래를 뒤집음
    //            QRgb pixel;
    //            if (photometric == PHOTOMETRIC_RGB) {
    //                // RGBA 이미지인 경우
    //                pixel = qRgba(TIFFGetR(color), TIFFGetG(color), TIFFGetB(color), TIFFGetA(color));
    //            } else if (photometric == PHOTOMETRIC_MINISBLACK && bitsPerSample == 8) {
    //                // 8비트 Grayscale 이미지인 경우
    //                qDebug() << "Here 8bit grayscale";
    //                pixel = qGray(color);
    //            } else if (photometric == PHOTOMETRIC_MINISBLACK && bitsPerSample == 16) {
    //                // 16비트 Grayscale 이미지 처리, QImage는 16비트 그레이스케일을 직접 지원하지 않으므로 8비트로 변환하거나 다른 방식을 고려해야 할 수 있음
    //                // 여기서는 간단하게 16비트 값을 8비트로 변환하는 예제를 제공합니다.
    //                pixel = qGray(color);
    //            } else {
    //                // 기타 photometric 처리
    //                pixel = qRgba(TIFFGetR(color), TIFFGetG(color), TIFFGetB(color), TIFFGetA(color));
    //            }
    //            image.setPixel(x, y, pixel);
    //        }
    //    }
    image.fill(0);
    for(int y=0; y < height; ++y){
        if(bitsPerSample == 16){
            uint16_t* scanline = reinterpret_cast<uint16_t*>(image.scanLine(y));
            auto val = TIFFReadScanline(tiff, scanline, y, 0);
            qDebug() << "Scanline is" << val << y << scanline[0] << image.scanLine(y)[0];
        }
        if(bitsPerSample == 8){
            uchar* scanline = reinterpret_cast<uchar*>(image.scanLine(y));
            TIFFReadScanline(tiff, scanline, y, 0);
        }
    }

    // Free allocated memory
    delete[] buffer;
    // Close TIFF file
    TIFFClose(tiff);

    return image;
}

}
#endif // TIFFREADER_H
