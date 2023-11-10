#ifndef QDC_H
#define QDC_H

#include <QObject>
#include <QDebug>

#include "Processing/Image.h"

namespace Qylon{
namespace QDC{
struct Region{
    struct Boundary{
        double width;
        double height;
    };
    struct Point{
        double x;
        double y;
    };
    Boundary boundary;
    Point center;
    std::list<double> points;
};
enum SequenceType{
    Sequence_SoftwareTrigger,
    Sequence_Threshold,
    Sequence_Morphology
};
struct ValueContainer{
    Image *image=nullptr;
    std::string *string=nullptr;
    QDC::Region region;
};

class Sequence;
class QDC : public QObject
{
    Q_OBJECT
public:
    explicit QDC(QObject *parent = nullptr);

    void run(Sequence* firstQueue);
    bool logicError(){
        return false;
    }
    Sequence *addSequence(SequenceType Type);

private:
    QList<Sequence*> sql;
    Sequence* firstQueue=nullptr;
    Image image;

signals:

};
}
}

#endif // QDC_H
