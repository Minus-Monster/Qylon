#ifndef GRAPHICSVIEW_H
#define GRAPHICSVIEW_H

#include "qcoreapplication.h"
#include "qgraphicssceneevent.h"
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

namespace Qylon{
class GraphicsView : public QGraphicsView {
    Q_OBJECT
public:
    GraphicsView();
    ~GraphicsView();
    void setRatio(float ratio);
    void setCrossHair(bool on, int width=3, QColor color=QColor(0,255,0));
    void setScale(float zoomPercent);
    void resetScale();
    bool isFit(){ return fitMode; }
    void setLogo(bool on){ logo = on; }
    void clear();


signals:
    void currentRatio(float ratio);

public slots:
    void setFit(bool on);

private:
    bool fitMode = false;
    bool crossHair = false;
    bool logo = true;
    bool drag = false;
    QGraphicsLineItem *lineH = nullptr;
    QGraphicsLineItem *lineV = nullptr;
    QTransform currentScale = QTransform(1,0,0,1,0,0);

protected:
    void resizeEvent(QResizeEvent *) override;
    void drawBackground(QPainter *painter, const QRectF &rect) override;
};


}

#endif // GRAPHICSVIEW_H
