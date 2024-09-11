#ifndef LINEPROFILEWIDGET_H
#define LINEPROFILEWIDGET_H
#include "Processing/ImageTools.h"
#include "qdebug.h"
#include <QToolTip>
#include <QTableWidget>
#include <QPainter>
#include <QPainterPath>
#include <QHeaderView>
#include <QPainterPathStroker>
#include <QGraphicsItem>
#include <QHBoxLayout>
#include <QMouseEvent>
#include "Modules/Graph.h"

namespace Qylon{
class GraphicsView;
class RulerItem : public QGraphicsLineItem{
public:
    RulerItem(const QLineF line=QLineF(), GraphicsView *parent=nullptr);
private:
    GraphicsView *parent;
protected:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    void hoverEnterEvent(QGraphicsSceneHoverEvent *event) override{}
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *event) override{}
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override{}
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override{}
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override{}
};
class LineProfileWidget : public QWidget{
    Q_OBJECT
public:
    LineProfileWidget(GraphicsView *view,QWidget *parent=nullptr) :
        QWidget(parent), ruler(new RulerItem(QLineF(), view)), graph(new Graph(this)), tableWidget(new QTableWidget(this)){
        setWindowTitle("Line Profile");
        setWindowIcon(QIcon(":/Resources/Icon.png"));

        setLayout(&layout);
        layout.addWidget(graph);
        layout.addWidget(tableWidget);
        graph->setMinimumSize(300, 100);
        graph->resize(600, 230);
        graph->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        tableWidget->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    }
    void setLine(QLineF line){
        ruler->setLine(line);
    }
    void setFirstPosition(QPointF point){
        firstPosition = point;
    }
    void setSecondPosition(QPointF point){
        setLine(QLineF(firstPosition, point));
    }
    void setImage(const QImage& image){
        graph->clear();

        tableWidget->clear();
        tableWidget->setRowCount(3);
        tableWidget->setColumnCount(2);
        tableWidget->setHorizontalHeaderLabels(QStringList() << "Property" << "Value");
        tableWidget->setItem(0, 0, new QTableWidgetItem("Distance"));
        tableWidget->setItem(0, 1, new QTableWidgetItem(QString::number(ruler->line().length())));
        tableWidget->setItem(1, 0, new QTableWidgetItem("Point 1:"));
        tableWidget->setItem(1, 1, new QTableWidgetItem("[" +QString::number((int)ruler->line().p1().x()) + ", " + QString::number((int)ruler->line().p1().y())+"]"));
        tableWidget->setItem(2, 0, new QTableWidgetItem("Point 2:"));
        tableWidget->setItem(2, 1, new QTableWidgetItem("[" +QString::number((int)ruler->line().p2().x()) + ", " + QString::number((int)ruler->line().p2().y())+"]"));
        tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
        tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        tableWidget->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);

        auto profile = getLineProfileData(image, ruler->line());
        graph->setGraphInformation("Euclidean Distance",0,(int)ruler->line().length(), "Pixel Value", 0,255);
        for(int i=0; i<profile.size(); ++i)
            graph->addPath(profile[i]);
        graph->update();
    }
    RulerItem* getRuler(){ return ruler;}

private:
    RulerItem *ruler = nullptr;
    Graph *graph= nullptr;

    QPointF firstPosition;
    QHBoxLayout layout;
    QTableWidget *tableWidget;
};
}

#endif // LINEPROFILEWIDGET_H
