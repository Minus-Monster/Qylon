#ifndef GRAPHICSVIEW_H
#define GRAPHICSVIEW_H

#include <QGraphicsRectItem>
#include <QDoubleSpinBox>
#include <QDebug>
#include <QObject>
#include <QGraphicsView>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QScrollBar>


namespace Qylon{
class GraphicsView : public QGraphicsView {
    Q_OBJECT
public:
    GraphicsView(){
        setMouseTracking(true);
        setFrameShape(Shape::StyledPanel);
    }
    void setRatio(float ratio){
        QTransform matrix = QTransform(ratio, 0, 0, ratio, 0, 0);
        setTransform(matrix);
        currentScale = matrix;
        emit currentRatio(this->transform().m11());
    }
    void setCrossHair(bool on){
        crossHair = on;
        if(on){
            if(lineH == nullptr) lineH = new QGraphicsLineItem;
            if(lineV == nullptr) lineV = new QGraphicsLineItem;

            QPen pen;
            pen.setColor(QColor(0,255,0));
            pen.setWidth(3);
            lineH->setPen(pen);
            lineV->setPen(pen);

            lineH->setLine(0,this->scene()->sceneRect().height()/2, this->scene()->sceneRect().width() ,this->scene()->sceneRect().height()/2);
            lineV->setLine(this->scene()->sceneRect().width()/2,0,this->scene()->sceneRect().width()/2, this->scene()->sceneRect().height());
            this->scene()->addItem(lineH);
            this->scene()->addItem(lineV);
        }else{
            this->scene()->removeItem(lineH);
            this->scene()->removeItem(lineV);
        }
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
    void removeBackground(){

    }

signals:
    void currentRatio(float ratio);

public slots:
    void setFit(bool on){
        fitMode = on;
        if(on) resizeEvent(nullptr);
        else setTransform(currentScale);
        emit currentRatio(this->transform().m11());
    }

private:
    bool fitMode = false;
    bool crossHair = false;
    QGraphicsLineItem *lineH = nullptr;
    QGraphicsLineItem *lineV = nullptr;
    QTransform currentScale = QTransform(1,0,0,1,0,0);

protected:
    void resizeEvent(QResizeEvent *) override{
        if(fitMode) fitInView(this->scene()->sceneRect(), Qt::KeepAspectRatio);
        if(crossHair){
            lineH->setLine(0,this->scene()->sceneRect().height()/2, this->scene()->sceneRect().width() ,this->scene()->sceneRect().height()/2);
            lineV->setLine(this->scene()->sceneRect().width()/2,0,this->scene()->sceneRect().width()/2, this->scene()->sceneRect().height());
        }
        updateSceneRect(this->viewport()->rect());
    }
    void drawBackground(QPainter *painter, const QRectF &rect) override{
        QGraphicsView::drawBackground(painter, rect);

        // Save the current painter transform
        QTransform originalTransform = painter->transform();

        // Temporarily reset the transform to ignore any scaling (zoom)
        painter->resetTransform();

        // Now calculate the position based on the original rect and the current zoom level
        QImage logo(":/Resources/Logo.png");
        QPointF center = originalTransform.map(rect.center()); // Map center using the original transform
        int x = center.x() - (logo.width() / 2.0);
        int y = center.y() - (logo.height() / 2.0);

        // Adjust coordinates to account for the current scroll position of the view
        x -= this->horizontalScrollBar()->value();
        y -= this->verticalScrollBar()->value();

        // Draw the pixmap without the scaling effect
        painter->drawPixmap(x, y, QPixmap::fromImage(logo));

        // Restore the original painter transform
        painter->setTransform(originalTransform);
    }
};


}

#endif // GRAPHICSVIEW_H
