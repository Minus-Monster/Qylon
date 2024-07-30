#include "GraphicsView.h"

Qylon::GraphicsView::GraphicsView(){
    setMouseTracking(true);
    setCursor(Qt::CrossCursor);
    setFrameShape(Shape::Box);
    setDragMode(DragMode::NoDrag);
}

Qylon::GraphicsView::~GraphicsView(){
}

void Qylon::GraphicsView::setRatio(float ratio){
    QTransform matrix = QTransform(ratio, 0, 0, ratio, 0, 0);
    setTransform(matrix);
    currentScale = matrix;
    emit currentRatio(this->transform().m11());
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
    updateSceneRect(this->viewport()->rect());
}

void Qylon::GraphicsView::drawBackground(QPainter *painter, const QRectF &rect){
    QGraphicsView::drawBackground(painter, rect);

    if(logo){
        QTransform originalTransform = painter->transform();
        painter->resetTransform();

        QImage logo(":/Resources/Logo.png");
        QPointF center = originalTransform.map(rect.center());
        int x = center.x() - (logo.width() / 2.0);
        int y = center.y() - (logo.height() / 2.0);

        x -= this->horizontalScrollBar()->value();
        y -= this->verticalScrollBar()->value();

        painter->drawPixmap(x, y, QPixmap::fromImage(logo));
        painter->setTransform(originalTransform);
    }
}
