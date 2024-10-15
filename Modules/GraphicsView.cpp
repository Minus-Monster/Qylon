#include "GraphicsView.h"
#include <qapplication.h>
Qylon::GraphicsView::GraphicsView(QWidget *parent) : QGraphicsView(parent){
    setMouseTracking(true);
    setFrameShape(Shape::Box);

    setCursor(Qt::CrossCursor);
    setDragMode(DragMode::NoDrag);
    setScene(&scene);
}

Qylon::GraphicsView::~GraphicsView(){
    if(lineProfileWidget) lineProfileWidget->deleteLater();
    if(histogramWidget) histogramWidget->deleteLater();
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

void Qylon::GraphicsView::setImage(const QImage &image){
    scene.setImage(image);
    scene.setSceneRect(0, 0, image.width(), image.height());

    if(histogramWidget) histogramWidget->setImage(image);
    if(lineProfileWidget) lineProfileWidget->setImage(image);
    if(lineH) lineH->setLine(0, scene.sceneRect().height()/2, scene.sceneRect().width(), scene.sceneRect().height()/2);
    if(lineV) lineV->setLine(scene.sceneRect().width()/2,0,scene.sceneRect().width()/2, scene.sceneRect().height());
}

bool Qylon::GraphicsView::addGraphicsItem(QGraphicsItem *item){
    QMutexLocker locker(&mutex);
    try{
        if(!item) return false;
        if(QGraphicsRectItem* rect = dynamic_cast<QGraphicsRectItem*>(item)){
            QBrush brush(rectangleColor);
            rect->setBrush(brush);
            rect->setPen(Qt::NoPen);
        }else if(QGraphicsEllipseItem *ellipse = dynamic_cast<QGraphicsEllipseItem*>(item)){
            QBrush brush(ellipseColor);
            ellipse->setBrush(brush);
            ellipse->setPen(Qt::NoPen);
        }else if(QGraphicsLineItem *line = dynamic_cast<QGraphicsLineItem*>(item)){
            QPen pen(outlineColor);
            pen.setWidth(2);
            line->setPen(pen);
        }else{
            qDebug() <<"Else case occurred";
            return false;
        }
        item->setVisible(graphicsItemVisible);
        scene.addItem(item);
    }catch(std::exception &e){
        qDebug() << "DrawGraphicsItem function :" << e.what();
        return false;
    }catch(...){
        qDebug() << "DrawGraphicsItem function :" << "Unknowned error occurred.";
        return false;
    }
    return true;
}

void Qylon::GraphicsView::removeGraphicsItem(QGraphicsItem *item){
    scene.removeItem(item);

}

void Qylon::GraphicsView::setGraphicsItemsVisible(bool on){
    for(int i=0; i<scene.items().size(); ++i){
        if(scene.items().at(i) == lineH || scene.items().at(i) == lineV) continue;
        if(scene.getPixmapItem() != scene.items().at(i)) scene.items().at(i)->setVisible(on);
    }
    graphicsItemVisible = on;
}

void Qylon::GraphicsView::setLineProfileMode(bool on){
    lineProfileMode = on;
    if(!on){
        if(lineProfileWidget && scene.isExisting(lineProfileWidget->getRuler())){
            scene.removeItem(lineProfileWidget->getRuler());
            this->viewport()->update();
        }
        if(lineProfileWidget) lineProfileWidget->deleteLater();
        lineProfileWidget = nullptr;
    }
}

void Qylon::GraphicsView::setHistogramMode(bool on)
{
    if(on){
        if(!histogramWidget){
            histogramWidget = new HistogramWidget;
            connect(histogramWidget, &HistogramWidget::closed, this,[this](){
                emit this->histogramWidgetClosed();
                setHistogramMode(false);
            });
        }
        if(!getCurrentImage().isNull()) histogramWidget->setImage(getCurrentImage());

        histogramWidget->show();
    }else{
        histogramWidget->hide();
        histogramWidget->deleteLater();
        histogramWidget = nullptr;
    }
}

void Qylon::GraphicsView::clear(){ // All clear on Graphics View
    QMutexLocker locker(&mutex);
    try{
        for(auto &item : scene.items()){
            if(item != scene.getPixmapItem()){
                if(item == lineV || item == lineH) continue;
                scene.removeItem(item);
                delete item;
            }else{
                scene.setImage(QImage());
            }
        }
    }catch(std::exception &e){
        qDebug() << "clear function :" << e.what();
    }catch(...){
        qDebug() << "clear function :" << "Unknowned error occurred.";
    }
}

void Qylon::GraphicsView::reset(){
    scene.setSceneRect(0,0,0,0);
    clear();
    setLogo(true);
}

void Qylon::GraphicsView::setCrossHair(bool on, QColor color, int width)
{
    if(on){
        if(!lineH) lineH = new QGraphicsLineItem;
        if(!lineV) lineV = new QGraphicsLineItem;
        QPen pen(color);
        pen.setWidth(width);
        lineH->setPen(pen);
        lineH->setLine(0, scene.sceneRect().height()/2, scene.sceneRect().width(), scene.sceneRect().height()/2);
        lineH->setZValue(10);
        lineV->setPen(pen);
        lineV->setLine(scene.sceneRect().width()/2,0,scene.sceneRect().width()/2, scene.sceneRect().height());
        lineV->setZValue(10);
        if(!scene.items().contains(lineH)) scene.addItem(lineH);
        if(!scene.items().contains(lineV)) scene.addItem(lineV);
    }else{
        delete lineH;
        delete lineV;
        lineH = nullptr;
        lineV = nullptr;
    }
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
    if(fitMode) fitInView(scene.sceneRect(), Qt::KeepAspectRatio);
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

void Qylon::GraphicsView::mousePressEvent(QMouseEvent *event){
    auto pos = mapToScene(event->pos());
    if((pos.x() <0 || pos.x() >= scene.sceneRect().width()) ||
        (pos.y() <0 || pos.y() >= scene.sceneRect().height())){
        return;
    }
    isDragging = true;
    clickedPos = pos;
    if(lineProfileMode){
        if(!lineProfileWidget) lineProfileWidget = new LineProfileWidget(this);
        auto ruler = lineProfileWidget->getRuler();
        if(!scene.isExisting(ruler)) scene.addItem(ruler);
        lineProfileWidget->setFirstPosition(clickedPos);
        lineProfileWidget->hide();
        this->viewport()->update();
    }else{
        setDragMode(DragMode::ScrollHandDrag);
        setCursor(Qt::ClosedHandCursor);

    }
    QGraphicsView::mousePressEvent(event);
}

void Qylon::GraphicsView::mouseMoveEvent(QMouseEvent *event){
    if(logo) return;
    auto pos = mapToScene(event->pos());
    if((pos.x() <0 || pos.x() >= scene.sceneRect().width()) ||
        (pos.y() <0 || pos.y() >= scene.sceneRect().height())){
        return;
    }
    emit currentPos(pos);
    if(lineProfileMode && isDragging){
        lineProfileWidget->setSecondPosition(pos);
        this->viewport()->update();
    }

    QGraphicsView::mouseMoveEvent(event);
}

void Qylon::GraphicsView::mouseReleaseEvent(QMouseEvent *event){
    auto pos = mapToScene(event->pos());
    if((pos.x() <0 || pos.x() >= scene.sceneRect().width()) ||
        (pos.y() <0 || pos.y() >= scene.sceneRect().height()) || pos == clickedPos){
        if(lineProfileWidget){
            lineProfileWidget->setLine(QLineF());
            lineProfileWidget->hide();
            if(scene.isExisting(lineProfileWidget->getRuler())) scene.removeItem(lineProfileWidget->getRuler());
            this->viewport()->update();
        }
        setDragMode(DragMode::NoDrag);
        setCursor(Qt::CrossCursor);
        isDragging = false;
        return;
    }

    if(lineProfileMode && isDragging){
        lineProfileWidget->setSecondPosition(pos);
        lineProfileWidget->setImage(getCurrentImage());
        lineProfileWidget->move(event->globalPos() + QPoint(15,15));
        lineProfileWidget->show();
        this->viewport()->update();
    }

    isDragging = false;

    setDragMode(DragMode::NoDrag);
    setCursor(Qt::CrossCursor);
    QGraphicsView::mouseReleaseEvent(event);
}
