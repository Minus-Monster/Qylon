#ifndef GRAPHICSWIDGET_H
#define GRAPHICSWIDGET_H
#include <QWidget>
#include <QSpacerItem>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QToolBar>
#include <QFileDialog>
#include <QStatusBar>
#include <QMessageBox>
#include <QImageReader>
#include <QCheckBox>
#include <QTimer>
#include "GraphicsView.h"
#include "GraphicsScene.h"

namespace Qylon{
class GraphicsWidgetSettings;
class GraphicsWidget : public QWidget{
    Q_OBJECT
public:
    GraphicsWidget(QWidget *parent = nullptr);
    ~GraphicsWidget();
    void initialize(bool isVTK=false);
    void setToolBarEnable(bool on);
    void setGraphicsItemsVisible(bool on);
    void clear();
    void setLogo(bool on);

signals:
    void updateWidget();

public slots:
    void setImage(const QImage image);
    void updateImage();
    void drawGraphicsItem(QGraphicsItem *item);
    void setFit(bool on);
    void setFPSEnable(bool on);
    void setBitShift(int val){
        bitShift = val;
    }
    void removeAllGraphicsItem();

#ifdef PCL_ENABLED
    void setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudData, QString pointCloudName);
#endif

private:
    GraphicsWidgetSettings *settings;
    GraphicsView *view;
    GraphicsScene *scene;
    QLabel *labelCoordinate;
    QLabel *labelPixelColor;
    QLabel *labelImageInformation;
    QLineEdit *lineEditPixelColor;
    QToolBar toolBar;
    QStatusBar statusBar;
    QImage currentImage;
    double currentRatioValue = 100.;

    QLabel labelFPS;
    QTimer fpsTimer;
    int timerCnt = 0;

    int bitShift = 0;
    QVBoxLayout layout;
    bool graphicsItemVisible=true;
};
class GraphicsWidgetSettings : public QDialog{
    Q_OBJECT
public:
    GraphicsWidgetSettings(QWidget *parent=nullptr) : QDialog(parent){
        setLayout(&layout);
        setWindowTitle("Basler vLauncher");
        Qt::WindowFlags flags = windowFlags();
        flags &= ~Qt::WindowContextHelpButtonHint;
        setWindowFlags(flags);

        QHBoxLayout *layoutBitShift = new QHBoxLayout;
        QLabel *labelBitShift = new QLabel("16-bit mono Image Shift:");
        QSpinBox *spinBoxBitShift = new QSpinBox;
        spinBoxBitShift->setRange(0, 7);
        spinBoxBitShift->setValue(0);
        connect(spinBoxBitShift, QOverload<int>::of(&QSpinBox::valueChanged), this, [&](int val){
            emit setBitShift(val);
            reinterpret_cast<GraphicsWidget*>(this->parent())->updateImage();
        });
        layoutBitShift->addWidget(labelBitShift);
        layoutBitShift->addWidget(spinBoxBitShift);
        layout.addLayout(layoutBitShift);

        QHBoxLayout *layoutGraphicsOn = new QHBoxLayout;
        QLabel *labelGraphicsOn = new QLabel("Graphics Layer:");
        QCheckBox *checkBoxGraphicsOn = new QCheckBox;
        checkBoxGraphicsOn->setChecked(true);
        connect(checkBoxGraphicsOn, &QCheckBox::clicked, this, [&](bool on){
            reinterpret_cast<GraphicsWidget*>(this->parent())->setGraphicsItemsVisible(on);
        });
        layoutGraphicsOn->addWidget(labelGraphicsOn);
        layoutGraphicsOn->addWidget(checkBoxGraphicsOn);
        layout.addLayout(layoutGraphicsOn);
    }
signals:
    void setBitShift(int value);

private:
    QVBoxLayout layout;
};
}
#endif // GRAPHICSWIDGET_H
