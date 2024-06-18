#ifndef TIFFREADER_H
#define TIFFREADER_H
#include <QImage>
#include <QDebug>
#ifdef TIFF_ENABLED
#include "tiffio.h"


namespace Qylon{
inline bool saveQImageToTiff16(const QImage &image, const QString &fileName) {
    if (image.format() != QImage::Format_Grayscale16) {
        qWarning() << "Image format must be QImage::Format_Grayscale16";
        return false;
    }

    TIFF *out = TIFFOpen(fileName.toStdString().c_str(), "w");
    if (!out) {
        qWarning() << "Could not open" << fileName << "for writing";
        return false;
    }

    int width = image.width();
    int height = image.height();

    TIFFSetField(out, TIFFTAG_IMAGEWIDTH, width);
    TIFFSetField(out, TIFFTAG_IMAGELENGTH, height);
    TIFFSetField(out, TIFFTAG_SAMPLESPERPIXEL, 1);
    TIFFSetField(out, TIFFTAG_BITSPERSAMPLE, 16);
    TIFFSetField(out, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
    TIFFSetField(out, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
    TIFFSetField(out, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);

    // Row buffer
    uint16_t *buf = new uint16_t[width];

    for (int y = 0; y < height; y++) {
        const uint16_t *scanline = reinterpret_cast<const uint16_t *>(image.scanLine(y));
        memcpy(buf, scanline, width * sizeof(uint16_t));
        if (TIFFWriteScanline(out, buf, y, 0) < 0) {
            delete[] buf;
            TIFFClose(out);
            qWarning() << "Failed to write scanline" << y;
            return false;
        }
    }

    delete[] buf;
    TIFFClose(out);
    return true;
}
inline const QImage openTiff(QString filePath){
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
    uint16_t photometric;
    TIFFGetField(tiff, TIFFTAG_PHOTOMETRIC, &photometric);
    uint32_t bitsPerSample = 0;
    TIFFGetField(tiff, TIFFTAG_BITSPERSAMPLE, &bitsPerSample);
    uint32_t order;
    TIFFGetField(tiff, TIFFTAG_FILLORDER, &order);
    QImage::Format imageFormat = QImage::Format_Invalid;
    uint32_t *buffer = new uint32_t[imageSize];
    if (!buffer) {
        qWarning() << "Failed to allocate memory for TIFF image";
        TIFFClose(tiff);
        return QImage();
    }

    switch (photometric) {
    case PHOTOMETRIC_RGB:
        imageFormat = (bitsPerSample==16) ? QImage::Format_RGBA64 : QImage::Format_RGBA8888;
        break;
    case PHOTOMETRIC_PALETTE:
        imageFormat = QImage::Format_Indexed8;
        break;
    case PHOTOMETRIC_MINISBLACK:
        imageFormat = (bitsPerSample==16) ? QImage::Format_Grayscale16 : QImage::Format_Grayscale8;
        break;
    }

    QImage image(width, height, imageFormat);
    for(unsigned int y=0; y < height; ++y){
        if(bitsPerSample == 16){
            quint16* scanline = reinterpret_cast<quint16*>(image.scanLine(y));
            TIFFReadScanline(tiff, scanline, y, 0);

            // if only the image is littleEndian;
            if(order == FILLORDER_MSB2LSB){
                for(uint32_t x=0; x < width; ++x){
                    quint16 pixel = scanline[x];
                    QByteArray data(reinterpret_cast<const char*>(&pixel), sizeof(pixel));
                    QDataStream stream(&data, QIODevice::ReadWrite);
                    stream.setByteOrder(QDataStream::BigEndian);
                    stream >> pixel;

                    scanline[x] = pixel;
                }
            }
        }
        if(bitsPerSample == 8){
            uchar* scanline = reinterpret_cast<uchar*>(image.scanLine(y));
            TIFFReadScanline(tiff, scanline, y, 0);
        }
    }
    delete[] buffer;
    TIFFClose(tiff);

    return image;
}

}
#endif
#endif // TIFFREADER_H
