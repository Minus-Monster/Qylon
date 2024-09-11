#ifndef GRAPHICSVIEW_H
#define GRAPHICSVIEW_H

#include <QGraphicsSceneEvent>
#include <QGraphicsRectItem>
#include <QDoubleSpinBox>
#include <QDebug>
#include <QObject>
#include <QGraphicsView>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QScrollBar>
#include <QMouseEvent>
#include <QMutex>

#include "Modules/GraphicsScene.h"
#include "Processing/LineProfileWidget.h"
#include "Processing/HistogramWidget.h"

namespace Qylon{
class GraphicsView : public QGraphicsView {
    Q_OBJECT
public:
    GraphicsView(QWidget *parent=nullptr);
    ~GraphicsView();
    void setRatio(float ratio);
    void setScale(float zoomPercent);
    void resetScale();
    bool isFit(){ return fitMode; }
    void setLogo(bool on){ logo = on; }
    void setImage(const QImage &image);
    const QImage getCurrentImage(){ return scene.getImage();}
    bool addGraphicsItem(QGraphicsItem *item);
    void removeGraphicsItem(QGraphicsItem *item);
    void setGraphicsItemsVisible(bool on);
    void setLineProfileMode(bool on);
    void setHistogramMode(bool on);
    void clear();
    void reset();
    void setCrossHair(bool on, QColor color, int width);

signals:
    void currentRatio(float ratio);
    void currentPos(QPointF point);
    void histogramWidgetClosed();

public slots:
    void setFit(bool on);

private:
    GraphicsScene scene;
    QMutex mutex;
    bool fitMode = false;
    bool graphicsItemVisible = true;
    bool lineProfileMode = false;
    bool isDragging = false;
    bool logo = true;
    LineProfileWidget *lineProfileWidget=nullptr;
    HistogramWidget *histogramWidget=nullptr;

    QTransform currentScale = QTransform(1,0,0,1,0,0);
    QColor rectangleColor = QColor(20,72,126,127);
    QColor ellipseColor = QColor(104, 189, 69,127);
    QColor outlineColor = QColor(249,157,51,127);

    QGraphicsLineItem *lineH = nullptr;
    QGraphicsLineItem *lineV = nullptr;
    QPointF clickedPos;

protected:
    void resizeEvent(QResizeEvent *) override;
    void drawBackground(QPainter *painter, const QRectF &rect) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

};
}

#endif // GRAPHICSVIEW_H
