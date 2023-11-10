#include "Processing/QDC.h"
#include "Processing/Sequence.h"
#ifdef OPENCV_ENABLED
Qylon::QDC::QDC::QDC(QObject *parent)
    : QObject{parent}
{


}

void Qylon::QDC::QDC::run(Sequence *firstQueue){

    Sequence *cur = firstQueue;

    /// Here to be an Image resource (original)
    /// and all of functions can be access with this resource;
    /// Image image;
    int i = 0;
    while( cur != nullptr ){
        cur->run();
        cur = cur->next();

        qDebug() << i++ ;
        if(cur->container.image != nullptr){
            // why do this code dead?
            cv::imshow(std::to_string(i), cur->container.image->toMat());
        }
    }
    return;
}

Qylon::QDC::Sequence *Qylon::QDC::QDC::addSequence(SequenceType Type){
    switch(Type){
    case SequenceType::Sequence_Threshold:{
        Threshold *thresh = new Threshold;
        return thresh;
    }
    case SequenceType::Sequence_Morphology:{
        break;
    }
    case SequenceType::Sequence_SoftwareTrigger:{
        SoftwareTrigger *soft = new SoftwareTrigger;
        return soft;
        break;
    }
    }
}
#endif

