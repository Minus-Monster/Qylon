#ifndef HISTOGRAMWIDGET_H
#define HISTOGRAMWIDGET_H

#include <QHeaderView>
#include <QObject>
#include <QWidget>
#include <QDebug>
#include <QVBoxLayout>
#include <QIcon>
#include <QTableWidget>
#include <limits>
#include <omp.h>

#include "Modules/Graph.h"
#include "Processing/ImageTools.h"

namespace Qylon{
class HistogramWidget : public QWidget{
    Q_OBJECT
public:
    HistogramWidget(QWidget *parent=nullptr):
        QWidget(parent), graph(new Graph), tableWidget(new QTableWidget){
        graph->setMinimumSize(300, 100);
        graph->resize(600, 230);
        graph->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        setWindowIcon(QIcon(":/Resources/Icon.png"));
        setWindowTitle("Histogram Analyzer");
        setLayout(&layout);
        layout.addWidget(graph);
        layout.addWidget(tableWidget);
        tableWidget->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        setWindowFlags(this->windowFlags() & ~Qt::WindowMinimizeButtonHint);
    }
    void setImage(const QImage &image){
        graph->clear();
        tableWidget->clear();

        std::vector<HistogramStatistic> info;
        auto histogram =getHistogramData(image, info);
        int minPixel = (std::numeric_limits<int>::max)();
        int maxPixel=0;
        int maxFrequency=0;
        tableWidget->setColumnCount(2);
        tableWidget->setRowCount(6*info.size());
        for(int i=0; i<info.size(); ++i){
            tableWidget->setItem(0+(6*i), 0, new QTableWidgetItem("Pixel (CH:" + QString::number(i+1)+")"));
            tableWidget->setItem(0+(6*i), 1, new QTableWidgetItem(QString::number(info[i].N)));
            tableWidget->setItem(1+(6*i), 0, new QTableWidgetItem("Min. Pixel (CH:" + QString::number(i+1)+")"));
            tableWidget->setItem(1+(6*i), 1, new QTableWidgetItem(QString::number(info[i].MinPixelValue)));
            tableWidget->setItem(2+(6*i), 0, new QTableWidgetItem("Max. Pixel (CH:" + QString::number(i+1)+")"));
            tableWidget->setItem(2+(6*i), 1, new QTableWidgetItem(QString::number(info[i].MaxPixelValue)));
            tableWidget->setItem(3+(6*i), 0, new QTableWidgetItem("Mean (CH:" + QString::number(i+1)+")"));
            tableWidget->setItem(3+(6*i), 1, new QTableWidgetItem(QString::number(info[i].Mean)));
            tableWidget->setItem(4+(6*i), 0, new QTableWidgetItem("Mode (CH:" + QString::number(i+1)+")"));
            tableWidget->setItem(4+(6*i), 1, new QTableWidgetItem(QString::number(info[i].Mode) + " (" + QString::number(info[i].MaxFrequency) +")"));
            tableWidget->setItem(5+(6*i), 0, new QTableWidgetItem("Std. Deviation (CH:" + QString::number(i+1)+")"));
            tableWidget->setItem(5+(6*i), 1, new QTableWidgetItem(QString::number(info[i].StdDev)));
            minPixel = (std::min)(minPixel, info[i].MinPixelValue);
            maxPixel = (std::max)(maxPixel, info[i].MaxPixelValue);
            maxFrequency = (std::max)(maxFrequency, info[i].MaxFrequency);
        }
        graph->setGraphInformation("Pixel Values", minPixel, maxPixel, "Frequency", 0, maxFrequency);
        for(int i=0; i<histogram.size(); ++i){
            graph->addPath(histogram[i]);
        }
        graph->update();

        tableWidget->setHorizontalHeaderLabels(QStringList() << "Property" << "Value");
        tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
        tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        tableWidget->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    }

signals:
    void closed();

private:
    Graph *graph;
    std::vector<std::vector<int>> histogram;

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
