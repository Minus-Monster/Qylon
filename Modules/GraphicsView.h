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
    GraphicsView();
    ~GraphicsView();
    void setRatio(float ratio);
    void setCrossHair(bool on);
    void setScale(float zoomPercent);
    void resetScale();
    bool isFit(){ return fitMode; }
    void setLogo(bool on){ logo = on; }
signals:
    void currentRatio(float ratio);

public slots:
    void setFit(bool on);

private:
    bool fitMode = false;
    bool crossHair = false;
    bool logo = true;
    QGraphicsLineItem *lineH = nullptr;
    QGraphicsLineItem *lineV = nullptr;
    QTransform currentScale = QTransform(1,0,0,1,0,0);

protected:
    void resizeEvent(QResizeEvent *) override;
    void drawBackground(QPainter *painter, const QRectF &rect) override;
};


}

#endif // GRAPHICSVIEW_H
