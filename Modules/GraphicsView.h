#ifndef GRAPHICSVIEW_H
#define GRAPHICSVIEW_H

#include <QDoubleSpinBox>
#include <QDebug>
#include <QObject>
#include <QGraphicsView>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>


namespace Qylon{
class GraphicsView : public QGraphicsView {
    Q_OBJECT
public:
    GraphicsView(){
        setMouseTracking(true);
    }
    void setRatio(float ratio){
        QTransform matrix = QTransform(ratio, 0, 0, ratio, 0, 0);
        setTransform(matrix);
        currentScale = matrix;
        emit currentRatio(this->transform().m11());
    }
    void setScale(float zoomPercent){ // 1 = 100%, 1.5 = 150%, 0.5 = 50%
        QTransform matrix = this->transform();
        matrix.scale(zoomPercent, zoomPercent);
        setTransform(matrix);
        currentScale = matrix;
        emit currentRatio(this->transform().m11());
    }
    void resetScale(){
        resetTransform();
        currentScale = this->transform();
        emit currentRatio(this->transform().m11());
    }
    bool isFit(){ return fitMode; }

signals:
    void currentRatio(float ratio);

public slots:
    void setFit(bool on){
        fitMode = on;
        if(on) resizeEvent(nullptr);
        else setTransform(currentScale);
//        currentScale = this->transform();
        emit currentRatio(this->transform().m11());
    }

private:
    bool fitMode = false;
    QTransform currentScale = QTransform(1,0,0,1,0,0);

protected:
    void resizeEvent(QResizeEvent *) override{
        if(fitMode) fitInView(this->scene()->sceneRect(), Qt::KeepAspectRatio);
        updateSceneRect(this->viewport()->rect());
    }
};


}

#endif // GRAPHICSVIEW_H
