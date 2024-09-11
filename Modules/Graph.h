#ifndef GRAPH_H
#define GRAPH_H
#include <QWidget>
#include <QObject>
#include <QImage>
#include <QPainterPath>
#include <QMouseEvent>
#include <QHeaderView>
#include <QDebug>
#include <QPainter>
#include <QToolTip>

namespace Qylon{
struct GraphInformation{
    QString xAxisName = "X";
    int xAxisMin =0;
    int xAxisMax =-1;
    QString yAxisName = "Y";
    int yAxisMin = 0;
    int yAxisMax =-1;
};

class Graph : public QWidget {
    Q_OBJECT

public:
    Graph(QWidget *parent = nullptr) : QWidget(parent) {
        setMouseTracking(true);
    }
    void setGraphInformation(GraphInformation info){
        this->info = info;
    }
    void setGraphInformation(QString xAxisName, int xAxisMin, int xAxisMax, QString yAxisName, int yAxisMin, int yAxisMax){
        this->info.xAxisName = xAxisName;
        this->info.xAxisMin = xAxisMin;
        this->info.xAxisMax = xAxisMax;
        this->info.yAxisName = yAxisName;
        this->info.yAxisMin = yAxisMin;
        this->info.yAxisMax = yAxisMax;
    }
    void addPath(std::vector<int> path){
        paths.push_back(path);
    }
    void clear(){
        paths.clear();
    }

protected:
    void paintEvent(QPaintEvent *event) override {
        QPainter painter(this);
        int width = this->width();
        int height = this->height();
        int xMin = (std::min)(info.xAxisMin, info.xAxisMax);
        int xMax = (std::max)(info.xAxisMin, info.xAxisMax);
        QRect graphRect(0 + margin*2, 0 + margin, width - margin*3, height - margin*3);
        painter.setBrush(QBrush(Qt::white));
        painter.setPen(Qt::lightGray);
        painter.drawRect(QRect(0,0,width, height));

        painter.setPen(QPen(Qt::black, 0.5, Qt::DotLine));
        QFontMetrics fm(painter.font());
        for(int i=0; i<4; ++i){
            // Intensity Line
            QString label = QString::number((info.xAxisMax-xMin+1)/4 * i + xMin);
            painter.drawText(graphRect.x()+(graphRect.width()/4)*i  - fm.horizontalAdvance(label)/2,
                             graphRect.bottom() + fm.height()*2+8,
                             label);
            if(i==0) continue;
            painter.drawLine(graphRect.x()+(graphRect.width()/4)*i,
                             graphRect.top(),
                             graphRect.x()+(graphRect.width()/4)*i,
                             graphRect.bottom());
            // Frequency Line
            label = QString::number(info.yAxisMax/3 * ((3-i)%3));
            painter.drawText(graphRect.left() - fm.horizontalAdvance(label) -8 - fm.height(), graphRect.top() + (graphRect.height()/3)*i + fm.height()/2, label);
            if(i==3) continue;
            painter.drawLine(graphRect.left(),
                             graphRect.top() + (graphRect.height()/3) *i,
                             graphRect.right(),
                             graphRect.top() + (graphRect.height()/3)*i);
        }
        QString label = QString::number(xMax);
        painter.drawText(graphRect.right() - fm.horizontalAdvance(label)/2, graphRect.bottom() + fm.height()*2+8, label);
        label = QString::number(info.yAxisMax);
        painter.drawText(graphRect.left() - fm.horizontalAdvance(label) -8 - fm.height(), graphRect.top() + fm.height()/2, label);

        int xAxisLabelWidth = fm.horizontalAdvance(info.xAxisName);
        painter.drawText(graphRect.center().x() - xAxisLabelWidth/2, graphRect.bottom() + fm.height() + 5, info.xAxisName);

        painter.save();
        painter.translate(graphRect.left()-fm.height(), graphRect.height()/2 + margin + fm.height()/2);
        painter.rotate(-90.);
        painter.drawText(-fm.horizontalAdvance(info.yAxisName)/2, fm.height()/2, info.yAxisName);
        painter.restore();

        painter.setPen(QPen(Qt::lightGray, 2));
        painter.setBrush(Qt::NoBrush);

        bool color = false;
        QVector<QColor> colors = {QColor(231, 76, 60,200), QColor(46, 204, 113,200), QColor(52, 152, 219,200)};
        if(paths.size() >=3) color=true;


        for(int channel=0; channel<paths.size(); ++channel){
            if(channel==3) continue;
            if(color){
                painter.setPen(QPen(colors[channel]));
            }
            // Drawing
            QPainterPath path;
            for (int i = xMin; i <= xMax; ++i) {
                int binHeight = (int)(((double)(paths[channel][i])/info.yAxisMax) * graphRect.height());
                int x = graphRect.left() + (int)(((double)(i-xMin)/(xMax-xMin)) * graphRect.width());
                int y = graphRect.bottom() - binHeight;

                if (i == xMin) path.moveTo(x, y);
                else path.lineTo(x, y);
            }
            painter.drawPath(path);
        }

        if (mousePosition.x() >= margin*2 && mousePosition.x() <= width - margin) {
            painter.setPen(QPen(Qt::lightGray, 1, Qt::DashLine));
            painter.drawLine(mousePosition.x()-1, graphRect.top(), mousePosition.x(), graphRect.bottom());
        }

        painter.setPen(QPen(Qt::black,1));
        painter.setBrush(Qt::NoBrush);
        QRect outBoundary = QRect(graphRect.x() - 1, graphRect.y()-1, graphRect.width()+2, graphRect.height()+2);
        painter.drawRect(outBoundary);
    }

    void mouseMoveEvent(QMouseEvent *event) override {
        int graphWidth = this->width() - margin *3;
        mousePosition = event->pos();

        if (mousePosition.x() >= margin && mousePosition.x() <= this->width() - margin) {
            int xMin = (std::min)(info.xAxisMin, info.xAxisMax);
            int xMax = (std::max)(info.xAxisMin, info.xAxisMax);
            int binIndex = (int)(((double)(mousePosition.x() - margin*2) / graphWidth) * (xMax-xMin))+xMin;
            if (binIndex >= xMin && binIndex <= xMax) {
                QString tooltipText = QString("%1: %2\n").arg(info.xAxisName).arg(binIndex);
                QVector<QString> channelCode = {"R", "G", "B"};

                for (int channel = 0; channel < paths.size(); ++channel) {
                    if(channel ==3) continue;
                    int binHeight = paths[channel][binIndex];
                    double normalizedY = static_cast<double>(binHeight) / info.yAxisMax;
                    int yValue = static_cast<int>(normalizedY * info.yAxisMax);

                    if(paths.size()==1) tooltipText += QString("%1: %3").arg(info.yAxisName).arg(yValue);
                    else tooltipText += QString("\n%1[%2]: %3").arg(info.yAxisName).arg(channelCode[channel]).arg(yValue);
                }

                QToolTip::showText(event->globalPos(), tooltipText, this);
            }
        }
        update();
    }


private:
    GraphInformation info;
    std::vector<std::vector<int>> paths;
    int margin = 35;

    QPoint mousePosition;
};
}

#endif // GRAPH_H
