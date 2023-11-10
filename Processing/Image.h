#ifndef IMAGE_H
#define IMAGE_H

#include <QImage>
#include <QDebug>
#ifdef OPENCV_ENABLED
#include <opencv2/imgproc.hpp>
#endif
namespace Qylon {

class Image {
public:
    enum Format {
        Format_Invalid,
        Format_Grayscale,
        Format_RGB888,
        Format_BGR888,
        Format_RGBA8888,
        Format_RGB32
    };
    struct Size{
        int width;
        int height;
    };

    Image()
        : _format(Format_Invalid), _width(0), _height(0), _bytesPerLine(0), _data(nullptr) {}

    Image(const QImage& image) {
        // This function will create an image with deep copy
        _format = Format_Invalid;

        _width = image.width();
        _height = image.height();
        _bytesPerLine = image.bytesPerLine();

        _data = new uchar[_height * _bytesPerLine];
        memcpy(_data, (uchar*)image.bits(), (_height * _bytesPerLine));

        switch (image.format()) {
        case QImage::Format_Grayscale8:
            _format = Format_Grayscale;
            break;
        case QImage::Format_RGB888:
            _format = Format_RGB888;
            break;
        case QImage::Format_BGR888:
            _format = Format_BGR888;
            break;
        case QImage::Format_RGBA8888:
            _format = Format_RGBA8888;
            break;
        case QImage::Format_RGB32:
            _format = Format_RGB32;
            break;
        default:
            qDebug() << "Invalid";
            break;
        }

        setData(_data, _width, _height, _format);
        qDebug() << _width << _height << _format;
    }

    Image(int width, int height, Format format)
        : _format(format), _width(width), _height(height) {
        // shallow copy
    }

    Image(uchar* data, int bytesPerLine, int width, int height, Format format)
        : _format(format), _width(width), _height(height), _bytesPerLine(bytesPerLine), _data(data) {
        // shallow copy
    }

    ~Image(){
        delete _data;
    }
    void setData(uchar* data, int width, int height, Format format) {
        _data = data;
        _width = width;
        _height = height;
        _format = format;
    }
#ifdef OPENCV_ENABLED
    Image(const cv::Mat& image, bool deepcopy=true) {
        // This function will create an image with deep copy
        int width = image.cols;
        int height = image.rows;
        int channels = image.channels();

        Format format;
        if (channels == 1) {
            format = Format_Grayscale;
        }
        else if (channels == 3) {
            format = Format_RGB888;
        }
        else if (channels == 4) {
            format = Format_RGBA8888;
        }
        else {
            format = Format_Invalid;
            // Handle invalid format here
        }
        size_t dataSize = image.total() * image.elemSize();
        if(deepcopy){
            _data = new uchar[dataSize];
            memcpy(_data, image.data, dataSize);
        }else{
            _data = image.data;
        }
        setData(_data, width, height, format);
        _bytesPerLine = image.step;
    }
    cv::Mat toMat() const{
        cv::Mat mat;
        switch (_format) {
        case Format_Grayscale:
            mat = cv::Mat(_height, _width, CV_8UC1, _data, _bytesPerLine);
            break;
        case Format_BGR888:
            mat = cv::Mat(_height, _width, CV_8UC3, _data, _bytesPerLine);
            break;
        case Format_RGB888:
            mat = cv::Mat(_height, _width, CV_8UC3, _data, _bytesPerLine);
            cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
            break;
        case Format_RGBA8888:
            mat = cv::Mat(_height, _width, CV_8UC4, _data, _bytesPerLine);
            cv::cvtColor(mat, mat, cv::COLOR_RGBA2BGR);
            break;
        case Format_RGB32:
            mat = cv::Mat(_height, _width, CV_8UC4, _data, _bytesPerLine);
            cv::cvtColor(mat, mat, cv::COLOR_BGRA2BGR);
            break;
        case Format_Invalid:
            qDebug() << "toMat Error";
            break;
        }

        return mat;
    }
    Image& operator=(cv::Mat &image){
        // This function will create an image with shallow copy
        int width = image.cols;
        int height = image.rows;
        int channels = image.channels();

        Format format;
        if (channels == 1) {
            format = Format_Grayscale;
        }
        else if (channels == 3) {
            format = Format_RGB888;
        }
        else if (channels == 4) {
            format = Format_RGBA8888;
        }
        else {
            format = Format_Invalid;
            // Handle invalid format here
        }
        _data = image.data;

        setData(_data, width, height, format);
        _bytesPerLine = image.step;

        return *this;
    }/*
    Image& operator=(cv::Mat *image){
        // This function will create an image with shallow copy
        int width = image->cols;
        int height = image->rows;
        int channels = image->channels();

        Format format;
        if (channels == 1) {
            format = Format_Grayscale;
        }
        else if (channels == 3) {
            format = Format_RGB888;
        }
        else if (channels == 4) {
            format = Format_RGBA8888;
        }
        else {
            format = Format_Invalid;
            // Handle invalid format here
        }
        _data = image->data;

        setData(_data, width, height, format);
        _bytesPerLine = image->step;
    }*/

    QImage toQImage() const{
        if (_format != Format_BGR888) {
            return QImage();
        }
        cv::Mat src = toMat();
        QImage dest((const uchar *)src.data, src.cols, src.rows, QImage::Format_RGB888);
        dest = dest.rgbSwapped();  // QImage stores RGB in reverse order
        return dest;
    }
#endif


private:
    Format _format;
    int _width;
    int _height;
    int _bytesPerLine;
    uchar* _data;
};
}


#endif // IMAGE_H
