#include "GraphicsView.h"

Qylon::GraphicsView::GraphicsView(){
    setMouseTracking(true);
    setCursor(Qt::CrossCursor);
    setFrameShape(Shape::Box);
    setDragMode(DragMode::NoDrag);
}

Qylon::GraphicsView::~GraphicsView(){
    deleteLater();
}

void Qylon::GraphicsView::setRatio(float ratio){
    QTransform matrix = QTransform(ratio, 0, 0, ratio, 0, 0);
    setTransform(matrix);
    currentScale = matrix;
    emit currentRatio(this->transform().m11());
}

void Qylon::GraphicsView::setCrossHair(bool on, int width, QColor color){
    crossHair = on;
    if(on){
        if(lineH == nullptr) lineH = new QGraphicsLineItem;
        if(lineV == nullptr) lineV = new QGraphicsLineItem;

        QPen pen;
        pen.setColor(color);
        pen.setWidth(width);
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

void Qylon::GraphicsView::setScale(float zoomPercent){ // 1 = 100%, 1.5 = 150%, 0.5 = 50%
    QTransform matrix = this->transform();
    matrix.scale(zoomPercent, zoomPercent);
    setTransform(matrix);
    currentScale = matrix;
    emit currentRatio(this->transform().m11());
}

void Qylon::GraphicsView::resetScale(){
    resetTransform();
    currentScale = this->transform();
    emit currentRatio(this->transform().m11());
}

void Qylon::GraphicsView::clear()
{

}

void Qylon::GraphicsView::setFit(bool on){
    fitMode = on;
    if(on)
        resizeEvent(nullptr);
    else
        setTransform(currentScale);

    emit currentRatio(this->transform().m11());
}

void Qylon::GraphicsView::resizeEvent(QResizeEvent *){
    if(fitMode) fitInView(this->scene()->sceneRect(), Qt::KeepAspectRatio);
    if(crossHair){
        lineH->setLine(0,this->scene()->sceneRect().height()/2, this->scene()->sceneRect().width() ,this->scene()->sceneRect().height()/2);
        lineV->setLine(this->scene()->sceneRect().width()/2,0,this->scene()->sceneRect().width()/2, this->scene()->sceneRect().height());
    }
    updateSceneRect(this->viewport()->rect());
}

void Qylon::GraphicsView::drawBackground(QPainter *painter, const QRectF &rect){
    QGraphicsView::drawBackground(painter, rect);

    if(logo){
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
}
