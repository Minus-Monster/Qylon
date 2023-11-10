#ifndef SEQUENCE_H
#define SEQUENCE_H
#include "Processing/Image.h"
#include "Processing/IO.h"
#include "Processing/QDC.h"
#include "Acquisition/Camera.h"

namespace Qylon{
namespace QDC{
class Sequence{
public:
    Sequence(){}
    ~Sequence(){}

    /// Run their own function.
    virtual void run()=0;

    Sequence* next(){
        return nextSeq;
    }
    void setNext(Sequence *next){
        nextSeq = next;
    }
    ValueContainer container;

private:
    Sequence *nextSeq=nullptr;
};
#ifdef OPENCV_ENABLED
class Threshold : public Sequence{
public:
    Threshold(){}
    void run() override{
        qDebug() << "Threshold sequence started.";

        cv::Mat input = _imageContainer->image->toMat();
        cv::Mat output;
        if(input.channels() != 1){
            cv::cvtColor(input, output, cv::COLOR_RGB2GRAY);
        }
        cv::threshold(output, output, _min, _max, _type);
        this->container.image = new Image(output);

        cv::imshow("B", this->container.image->toMat());
    }
    void set(){
        setMinimum(_min);
        setMaximum(_max);
        setThresholdType(_type);

        // Interacting parameteres are as below
//        setImage();
    }
    void setImage(ValueContainer *value){
        _imageContainer = value;
    }

    void setMinimum(double val){
        _min = val;
    }
    void setMaximum(double val){
        _max = val;
    }
    void setThresholdType(cv::ThresholdTypes type){
        _type = type;
    }

private:
    double _min = 127.;
    double _max = 255.;
    cv::ThresholdTypes _type = cv::ThresholdTypes::THRESH_BINARY;
    ValueContainer *_imageContainer;
};


class SoftwareTrigger : public Sequence{
public:
    SoftwareTrigger(){}
    void run() override{
        qDebug() << "Software trigger started.";
        this->container.image = new Image(QImage("/home/minwoo/teams.png"));
        cv::imshow("A", this->container.image->toMat());

    }
    void setCamera(Camera camera){
        if(camera.isOpened()){

        }else{
            qDebug() << "Camera is not opened. please connect your camera first.";
        }
    }
};
#endif
}
}
#endif // SEQUENCE_H
