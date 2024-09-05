#ifndef HISTOGRAMWIDGET_H
#define HISTOGRAMWIDGET_H

#include <QObject>
#include <QWidget>
#include <QPainter>
#include <QPen>
#include <QFontMetrics>
#include <QMouseEvent>
#include <QToolTip>
#include <QVector>
#include <QRect>
#include <QDebug>
#include <QPainterPath>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QIcon>
#include <QTreeWidget>

namespace Qylon{
class HistogramGraph : public QWidget {
    Q_OBJECT

public:
    HistogramGraph(QWidget *parent = nullptr) : QWidget(parent) {
        setMouseTracking(true);
    }

    void setImage(const QImage& image){
        if(image.pixelFormat().channelCount() == 0){
            qDebug() << "Histogram Analyzing error. Channel size is 0";
        }
        histogram.clear();
        grayscale = image.isGrayscale();
        colorChannel = image.pixelFormat().channelCount() - image.pixelFormat().alphaUsage();
        bitDepth = image.depth()/image.pixelFormat().channelCount();

        histogram.resize(colorChannel);
        for(int i=0; i<colorChannel; ++i){
            histogram[i].fill(0, 1<<bitDepth);
        }

        for(int y = 0; y < image.height(); ++y){
            if(grayscale){
                if(bitDepth == 8){
                    const uchar* line = image.scanLine(y);
                    for(int x = 0; x < image.width(); ++x) histogram[0][line[x]]++;
                }
                else if(bitDepth == 16){
                    const quint16* line = reinterpret_cast<const quint16*>(image.scanLine(y));
                    for(int x = 0; x < image.width(); ++x) histogram[0][line[x]]++;
                } else {
                    qDebug() << "Unsupported type image.";
                }
            } else {
                const uchar* line = image.scanLine(y);
                for (int x = 0; x < image.width(); ++x) {
                    QColor color = QColor::fromRgb(reinterpret_cast<const QRgb*>(line)[x]);
                    histogram[0][color.red()]++;
                    histogram[1][color.green()]++;
                    histogram[2][color.blue()]++;
                }
            }
        }
        update();
    }

protected:
    void paintEvent(QPaintEvent *event) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        int width = this->width();
        int height = this->height();
        QRect graphRect(0 + margin, 0 + margin, width - margin*2, height - margin*2 - 30);

        painter.setBrush(QBrush(Qt::white));
        painter.setPen(Qt::lightGray);
        painter.drawRect(QRect(0,0,width, height));

        painter.setPen(QPen(Qt::black, 0.5, Qt::DotLine));
        for(int i=1; i<4; ++i){
            painter.drawLine(graphRect.x()+(graphRect.width()/4)*i,
                             graphRect.top(),
                             graphRect.x()+(graphRect.width()/4)*i,
                             graphRect.bottom());
            if(i==3) continue;
            painter.drawLine(graphRect.left(),
                             graphRect.top() + (graphRect.height()/3) *i,
                             graphRect.right(),
                             graphRect.top() + (graphRect.height()/3)*i);
        }

        painter.setPen(QPen(Qt::lightGray, 2));
        painter.setBrush(Qt::NoBrush);

        int maxCount = 0;
        int numBins = 1 << bitDepth;
        if (grayscale && !histogram.isEmpty()) {
            maxCount = *std::max_element(histogram[0].begin(), histogram[0].end());
            QPainterPath path;
            for (int i = 0; i < numBins; ++i) {
                int binHeight = static_cast<int>((static_cast<double>(histogram[0][i]) / maxCount) * graphRect.height());
                int x = graphRect.left() + static_cast<int>((static_cast<double>(i) / (numBins - 1)) * graphRect.width());
                int y = graphRect.bottom() - binHeight;

                if (i == 0) path.moveTo(x, y);
                else path.lineTo(x, y);
            }
            painter.drawPath(path);
        }else if(!grayscale && !histogram.isEmpty()){
            QVector<QColor> colors = {QColor(231, 76, 60,200), QColor(46, 204, 113,200), QColor(52, 152, 219,200)};
            maxCount = std::max(
                std::max(*std::max_element(histogram[0].begin(), histogram[0].end()), *std::max_element(histogram[1].begin(), histogram[1].end())),
                *std::max_element(histogram[2].begin(), histogram[2].end()));
            for (int channel = 0; channel < colorChannel; ++channel) {
                QPainterPath path;
                painter.setPen(QPen(colors[channel]));

                for (int i = 0; i < numBins; ++i) {
                    int binHeight = static_cast<int>((static_cast<double>(histogram[channel][i]) / maxCount) * graphRect.height());
                    int x = graphRect.left() + static_cast<int>((static_cast<double>(i) / (numBins - 1)) * graphRect.width());
                    int y = graphRect.bottom() - binHeight;

                    if (i == 0) path.moveTo(x, y);
                    else path.lineTo(x, y);
                }
                painter.drawPath(path);
            }
        }else{} // DO NOTHING

        // Pivot
        painter.setPen(QPen(Qt::black, 0.5, Qt::DotLine));
        QFontMetrics fm(painter.font());
        QString label = QString::number(0); // X Minimum
        painter.drawText(graphRect.left() - fm.horizontalAdvance(label)/2, graphRect.bottom() + 20, label);
        label = QString::number(numBins-1); // X Maximum
        painter.drawText(graphRect.right() - fm.horizontalAdvance(label)/2, graphRect.bottom() + 20, label);
        for(int i=1; i< 4; ++i){
            int value = (int)((double)numBins/4) * i;
            int x = graphRect.left() + (int)((double)(value)/(double)numBins * graphRect.width());
            label = QString::number(value);
            painter.drawText(x - fm.horizontalAdvance(label) / 2, graphRect.bottom() + 20, label);
        }
        int xAxisLabelWidth = fm.horizontalAdvance("Intensity");
        painter.drawText((width - xAxisLabelWidth)/2, height-15, "Intensity");

        if (mousePosition.x() >= margin && mousePosition.x() <= width - margin) {
            painter.setPen(QPen(Qt::lightGray, 1, Qt::DashLine));
            painter.drawLine(mousePosition.x(), graphRect.top(), mousePosition.x(), graphRect.bottom());
        }
        painter.setPen(QPen(Qt::black,1));
        painter.setBrush(Qt::NoBrush);
        QRect outBoundary = QRect(graphRect.x() - 1, graphRect.y()-1, graphRect.width()+2, graphRect.height()+2);
        painter.drawRect(outBoundary);
    }

    void mouseMoveEvent(QMouseEvent *event) override {
        if(bitDepth==0) return;

        int graphWidth = this->width() - 2 * margin;
        mousePosition = event->pos();

        if (mousePosition.x() >= margin && mousePosition.x() <= this->width() - margin) {
            if(grayscale){
                int binIndex = static_cast<int>((static_cast<double>(mousePosition.x() - margin) / graphWidth) * histogram[0].size());
                if (binIndex >= 0 && binIndex < histogram[0].size()) {
                    int count = histogram[0][binIndex];
                    QString tooltipText = QString("Intensity: %1\nCount: %2").arg(binIndex).arg(count);
                    QToolTip::showText(event->globalPos(), tooltipText, this);
                }
            }else{
                int binIndex = static_cast<int>((static_cast<double>(mousePosition.x() - margin) / graphWidth) * histogram[0].size());
                if(binIndex >=0 && binIndex < histogram[0].size()){
                    int countR = histogram[0][binIndex];
                    int countG = histogram[1][binIndex];
                    int countB = histogram[2][binIndex];
                    QString tooltipText =
                        QString("Intensity: %1\n\nRed Count: %2\nGreen Count: %3\nBlue Count: %4")
                            .arg(binIndex).arg(countR).arg(countG).arg(countB);
                    QToolTip::showText(event->globalPos(), tooltipText, this);
                }
            }
            update();
        }
    }


private:
    QVector<QVector<int>> histogram;

    int bitDepth = 0;
    int colorChannel = 0;
    int margin = 20;

    QPoint mousePosition;
    bool grayscale = false;
};

class HistogramWidget : public QWidget{
    Q_OBJECT
public:
    HistogramWidget(QWidget *parent=nullptr):
        QWidget(parent), graph(new HistogramGraph), treeWidget(new QTreeWidget){
        graph->setMinimumSize(300, 100);
        graph->resize(500, 150);
        graph->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        setWindowTitle("Histogram Analyzer");
        setLayout(&layout);
        layout.addWidget(graph);
        layout.addWidget(treeWidget);
        treeWidget->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        treeWidget->setHeaderLabels(QStringList()<< "Description" << "Value");
        QList<QTreeWidgetItem*> items;
        items << new QTreeWidgetItem(QStringList() << "Min" << "0")
              << new QTreeWidgetItem(QStringList() << "Max" << "0")
              << new QTreeWidgetItem(QStringList() << "Mean" << "0")
              << new QTreeWidgetItem(QStringList() << "Mode" << "0")
              << new QTreeWidgetItem(QStringList() << "Std. Deviation" << "0");

        treeWidget->addTopLevelItems(items);
        setWindowFlags(this->windowFlags() & ~Qt::WindowMinimizeButtonHint);
    }
    void setImage(const QImage &image){
        graph->setImage(image);

        // Get data from Analyzer

    }
signals:
    void closed();
private:
    HistogramGraph *graph;
    QHBoxLayout layout;
    QTreeWidget *treeWidget;


protected:
    void closeEvent(QCloseEvent *event) override{
        emit closed();
        event->accept();
    }
};
}

#endif // HISTOGRAMWIDGET_H
