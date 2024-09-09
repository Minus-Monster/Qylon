#ifndef HISTOGRAMWIDGET_H
#define HISTOGRAMWIDGET_H

#include "qheaderview.h"
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
#include <QTableWidget>
#include <omp.h>


namespace Qylon{
struct ImageInformation{
    const QImage *image;
    double mean;
    double stdDev;
    int totalPixels;
    int maxFrequency;
    int minIntensity;
    int maxIntensity;
    int mode;
};

class HistogramGraph : public QWidget {
    Q_OBJECT

public:
    HistogramGraph(QWidget *parent = nullptr) : QWidget(parent) {
        setMouseTracking(true);
    }

    void setImage(const QImage& image){
        ImageInformation info;
        info.image = &image;

        if(image.pixelFormat().channelCount() == 0){
            qDebug() << "Histogram Analyzing error. Channel size is 0";
        }
        histogram.clear();
        grayscale = image.isGrayscale();
        colorChannel = image.pixelFormat().channelCount() - (grayscale ? 0 : image.pixelFormat().alphaUsage());
        bitDepth = image.depth()/image.pixelFormat().channelCount();
        histogram.resize(colorChannel);

        long long sum=0;
        long long sumSquaredDiffs = 0;
        int totalPixels = 0;
        int maxFrequency = 0;
        int minValue = -1;
        int maxValue = 0;
        int mode = 0;
        int maxFreqCalc = 0;

        for(int i=0; i<colorChannel; ++i){
            histogram[i].fill(0, 1<<bitDepth);
        }
#pragma omp parallel for reduction(+:sum, totalPixels) reduction(max:maxFrequency, maxValue) reduction(min:minValue)
        for(int y = 0; y < image.height(); ++y){
            if(grayscale){
                const void* line = image.scanLine(y);
                for(int x = 0; x < image.width(); ++x){
                    int value;
                    if(bitDepth ==8) value = reinterpret_cast<const uchar*>(line)[x];
                    else value = reinterpret_cast<const quint16*>(line)[x];
#pragma omp atomic
                    histogram[0][value]++;
                    sum += value;
                    totalPixels++;
#pragma omp critical
                    {
                        if(minValue ==- 1 || value < minValue) minValue = value;
                        if(value > maxValue){ maxValue = value;}
                        if(histogram[0][value] > maxFrequency){
                            maxFrequency = histogram[0][value];
                            mode = value;
                        }
                    }
                }
            } else {
                const uchar* line = image.scanLine(y);
                for (int x = 0; x < image.width(); ++x) {
                    QColor color = QColor::fromRgb(reinterpret_cast<const QRgb*>(line)[x]);                    
                    int rgb[3] = {color.red(),color.green(),color.blue()};

                    for(int colorCount =0; colorCount < colorChannel; ++colorCount){
                        int value = rgb[colorCount];
                        totalPixels++;
#pragma omp atomic
                        histogram[colorCount][value]++;
                        sum+= rgb[colorCount];
#pragma omp critical
                        {
                            if(minValue == -1 || value < minValue) minValue = value;
                            if(value > maxValue) maxValue = value;
                            if(histogram[colorCount][value] > maxFrequency){
                                maxFrequency = histogram[colorCount][value];
                                maxFreqCalc= histogram[0][rgb[colorCount]] + histogram[1][rgb[colorCount]] + histogram[2][rgb[colorCount]];
                                mode = value;
                            }
                        }
                    }

                }
            }
        }
        double mean = static_cast<double>(sum) / totalPixels;
        if(grayscale) {
#pragma omp parallel for reduction(+:sumSquaredDiffs)
            for(int i = 0; i < histogram[0].size(); ++i) {
                if(histogram[0][i] > 0) sumSquaredDiffs += histogram[0][i] * pow(i - mean, 2);
            }
        }else{
#pragma omp parallel for reduction(+:sumSquaredDiffs)
            for(int channel = 0; channel < 3; ++channel) {
                for(int i = 0; i < histogram[channel].size(); ++i) {
                    if(histogram[channel][i] > 0) sumSquaredDiffs += histogram[channel][i] * pow(i - mean, 2);
                }
            }
        }
        double stdDev = sqrt(static_cast<double>(sumSquaredDiffs) / totalPixels);
        info.mean = mean;
        info.stdDev = stdDev;
        info.mode = mode;
        info.minIntensity = minValue;
        info.maxIntensity = maxValue;
        info.maxFrequency = (grayscale? maxFrequency : maxFreqCalc);
        info.totalPixels = totalPixels;
        emit sendImageInformation(info);
        currentInformation = info;

        update();
    }

signals:
    void sendImageInformation(ImageInformation info);

protected:
    void paintEvent(QPaintEvent *event) override {
        QPainter painter(this);

        int width = this->width();
        int height = this->height();
        QRect graphRect(0 + margin*2, 0 + margin, width - margin*3, height - margin*2 - 30);
        painter.setBrush(QBrush(Qt::white));
        painter.setPen(Qt::lightGray);
        painter.drawRect(QRect(0,0,width, height));


        painter.setPen(QPen(Qt::black, 0.5, Qt::DotLine));
        QFontMetrics fm(painter.font());
        int maxFrequency = currentInformation.maxFrequency;
        int maxIntensity = currentInformation.maxIntensity;
        int minIntensity = currentInformation.minIntensity;
        int numBins = 1 << bitDepth;
        for(int i=0; i<4; ++i){
            // Intensity Line
            QString label = QString::number((maxIntensity-minIntensity+1)/4 * i + minIntensity );
            painter.drawText(graphRect.x()+(graphRect.width()/4)*i  - fm.horizontalAdvance(label)/2,
                             graphRect.bottom() + fm.height()*2+8,
                             label);
            if(i==0) continue;
            painter.drawLine(graphRect.x()+(graphRect.width()/4)*i,
                             graphRect.top(),
                             graphRect.x()+(graphRect.width()/4)*i,
                             graphRect.bottom());
            // Frequency Line
            label = QString::number(maxFrequency/3 * ((3-i)%3));
            painter.drawText(graphRect.left() - fm.horizontalAdvance(label) -8 - fm.height(), graphRect.top() + (graphRect.height()/3)*i + fm.height()/2, label);
            if(i==3) continue;
            painter.drawLine(graphRect.left(),
                             graphRect.top() + (graphRect.height()/3) *i,
                             graphRect.right(),
                             graphRect.top() + (graphRect.height()/3)*i);
        }
        QString label = QString::number(maxIntensity);
        painter.drawText(graphRect.right() - fm.horizontalAdvance(label)/2, graphRect.bottom() + fm.height()*2+8, label);
        label = QString::number(maxFrequency);
        painter.drawText(graphRect.left() - fm.horizontalAdvance(label) -8 - fm.height(), graphRect.top() + fm.height()/2, label);

        int xAxisLabelWidth = fm.horizontalAdvance("Intensity");
        painter.drawText(graphRect.center().x() - xAxisLabelWidth/2, graphRect.bottom() + fm.height() + 5, "Intensity");


        // Draw the "Frequency" text centered at the new origin
        QString yAxisLabel = "Frequency";
        painter.save();
        painter.translate(graphRect.left()-fm.height(), graphRect.height()/2 + margin + fm.height()/2);
        painter.rotate(-90.);
        painter.drawText(-fm.horizontalAdvance(yAxisLabel)/2, fm.height()/2, yAxisLabel);
        painter.restore();


        painter.setPen(QPen(Qt::lightGray, 2));
        painter.setBrush(Qt::NoBrush);
        if (grayscale && !histogram.isEmpty()) {
            QPainterPath path;
            for (int i = minIntensity; i <= maxIntensity; ++i) {
                int binHeight = static_cast<int>((static_cast<double>(histogram[0][i]) / maxFrequency) * graphRect.height());
                int x = graphRect.left() + static_cast<int>((static_cast<double>(i - minIntensity) / (maxIntensity - minIntensity)) * graphRect.width());
                int y = graphRect.bottom() - binHeight;

                if (i == minIntensity) path.moveTo(x, y);
                else path.lineTo(x, y);
            }
            painter.drawPath(path);
        }else if(!grayscale && !histogram.isEmpty()){
            QVector<QColor> colors = {QColor(231, 76, 60,200), QColor(46, 204, 113,200), QColor(52, 152, 219,200)};
            for (int channel = 0; channel < colorChannel; ++channel) {
                QPainterPath path;
                painter.setPen(QPen(colors[channel]));

                for (int i = 0; i < numBins; ++i) {
                    int binHeight = static_cast<int>((static_cast<double>(histogram[channel][i]) / maxFrequency) * graphRect.height());
                    int x = graphRect.left() + static_cast<int>((static_cast<double>(i) / (numBins - 1)) * graphRect.width());
                    int y = graphRect.bottom() - binHeight;

                    if (i == 0) path.moveTo(x, y);
                    else path.lineTo(x, y);
                }
                painter.drawPath(path);
            }
        }else{} // DO NOTHING


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
        if(bitDepth==0) return;

        int graphWidth = this->width() - margin *3;
        mousePosition = event->pos();

        if(grayscale){
            if (mousePosition.x() >= margin && mousePosition.x() <= this->width() - margin) {
                int binIndex = static_cast<int>((static_cast<double>(mousePosition.x() - margin*2) / graphWidth) * (currentInformation.maxIntensity-currentInformation.minIntensity)) +currentInformation.minIntensity;
                if (binIndex >= currentInformation.minIntensity && binIndex <= currentInformation.maxIntensity) {
                    int count = histogram[0][binIndex];
                    QString tooltipText = QString("Intensity: %1\nFrequency: %2").arg(binIndex).arg(count);
                    QToolTip::showText(event->globalPos(), tooltipText, this);
                }
            }
        }else{
            if (mousePosition.x() >= margin * 2 && mousePosition.x() <= this->width() - margin) {
                int binIndex = static_cast<int>((static_cast<double>(mousePosition.x() - margin*2) / graphWidth) * histogram[0].size());
                if(binIndex >=0 && binIndex < histogram[0].size()){
                    int countR = histogram[0][binIndex];
                    int countG = histogram[1][binIndex];
                    int countB = histogram[2][binIndex];
                    QString tooltipText =
                        QString("Intensity: %1\n\nFrequency(R): %2\nFrequency(G): %3\nFrequency(B): %4")
                            .arg(binIndex).arg(countR).arg(countG).arg(countB);
                    QToolTip::showText(event->globalPos(), tooltipText, this);
                }
            }
        }
        update();
    }


private:
    QVector<QVector<int>> histogram;
    QVector<QVector<QPainterPath>> graphPath;
    ImageInformation currentInformation;

    int bitDepth = 0;
    int colorChannel = 0;
    int margin = 30;

    QPoint mousePosition;
    bool grayscale = false;
};

class HistogramWidget : public QWidget{
    Q_OBJECT
public:
    HistogramWidget(QWidget *parent=nullptr):
        QWidget(parent), graph(new HistogramGraph), tableWidget(new QTableWidget){
        graph->setMinimumSize(300, 100);
        graph->resize(600, 230);
        graph->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        setWindowTitle("Histogram Analyzer");
        setLayout(&layout);
        layout.addWidget(graph);
        layout.addWidget(tableWidget);
        tableWidget->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        setWindowFlags(this->windowFlags() & ~Qt::WindowMinimizeButtonHint);

        connect(graph, &HistogramGraph::sendImageInformation, this, [=](ImageInformation info){
            tableWidget->clear();
            tableWidget->setRowCount(6);
            tableWidget->setColumnCount(2);
            tableWidget->setHorizontalHeaderLabels(QStringList() << "Property" << "Value");
            tableWidget->setItem(0, 0, new QTableWidgetItem("Pixel"));
            tableWidget->setItem(0, 1, new QTableWidgetItem(QString::number(info.totalPixels)));
            tableWidget->setItem(1, 0, new QTableWidgetItem("Min. Intensity"));
            tableWidget->setItem(1, 1, new QTableWidgetItem(QString::number(info.minIntensity)));
            tableWidget->setItem(2, 0, new QTableWidgetItem("Max. Intensity"));
            tableWidget->setItem(2, 1, new QTableWidgetItem(QString::number(info.maxIntensity)));
            tableWidget->setItem(3, 0, new QTableWidgetItem("Mean"));
            tableWidget->setItem(3, 1, new QTableWidgetItem(QString::number(info.mean)));
            tableWidget->setItem(4, 0, new QTableWidgetItem("Mode"));
            tableWidget->setItem(4, 1, new QTableWidgetItem(QString::number(info.mode) + " (" + QString::number(info.maxFrequency) +")"));
            tableWidget->setItem(5, 0, new QTableWidgetItem("Std. Deviation"));
            tableWidget->setItem(5, 1, new QTableWidgetItem(QString::number(info.stdDev)));
            tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
            tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
            tableWidget->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        });
    }
    void setImage(const QImage &image){
        graph->setImage(image);
    }

signals:
    void closed();

private:
    HistogramGraph *graph;
    QHBoxLayout layout;
    QTableWidget *tableWidget;


protected:
    void closeEvent(QCloseEvent *event) override{
        emit closed();
        event->accept();
    }
};
}

#endif // HISTOGRAMWIDGET_H
