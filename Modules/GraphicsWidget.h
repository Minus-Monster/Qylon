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
#include <QColorDialog>
#include <QGroupBox>
#include <QMutex>
#include <QTimer>
#include "GraphicsView.h"
#include "GraphicsScene.h"
#include "Processing/HistogramWidget.h"

namespace Qylon{
class GraphicsWidgetSettings;
class GraphicsWidget : public QWidget{
    Q_OBJECT
public:
    GraphicsWidget(QWidget *parent = nullptr);
    ~GraphicsWidget();
    void initialize();
    void setToolBarEnable(bool on);
    void setGraphicsItemsVisible(bool on);
    void clear();   // Remove all items from scene
    void reset();   // Remove all items from scene including sceneRect
    void setLogo(bool on);
    void setCrossHair(bool on, QColor color = QColor(104,189,69, 180), int width = 3);
    bool IsCrosshairEnabled(){ return crosshair; }
    void setRectColor(QColor color){ rectangleColor = color; }
    void setEllipseColor(QColor color){ ellipseColor = color; }
    void setOutlineColor(QColor color){ outlineColor = color; }

signals:
    void updateWidget();

public slots:
    void setImage(const QImage image);
    void updateImage();
    bool drawGraphicsItem(QGraphicsItem *item);
    void setFit(bool on);
    void setFPSEnable(bool on);
    void setBitShift(int val){ bitShift = val; }
    void removeAllGraphicsItem();

private:
    GraphicsWidgetSettings *settings;
    GraphicsView *view;
    GraphicsScene *scene;
    HistogramWidget *histogramWidget=nullptr;

    QLabel *labelCoordinateX;
    QLabel *labelCoordinateY;
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

    bool graphicsItemVisible=true;

    QGraphicsLineItem *lineH = nullptr;
    QGraphicsLineItem *lineV = nullptr;
    bool crosshair = false;

    QColor rectangleColor = QColor(20,72,126,127);
    QColor ellipseColor = QColor(104, 189, 69,127);
    QColor outlineColor = QColor(249,157,51,127);
    QVBoxLayout layout;
    int bitShift = 0;
    QMutex mutex;
};

class GraphicsWidgetSettings : public QDialog{
    Q_OBJECT
public:
    GraphicsWidgetSettings(QWidget *parent=nullptr) : QDialog(parent){
        setLayout(&layout);
        setWindowTitle("Graphics Engine Configuration");
        Qt::WindowFlags flags = windowFlags();
        flags &= ~Qt::WindowContextHelpButtonHint;
        setWindowFlags(flags);

        QGroupBox *groupBoxDrawingOption = new QGroupBox("Drawing Option");
        QVBoxLayout *layoutDrawingOption = new QVBoxLayout;
        groupBoxDrawingOption->setLayout(layoutDrawingOption);
        groupBoxDrawingOption->setCheckable(true);
        groupBoxDrawingOption->setChecked(true);
        connect(groupBoxDrawingOption, &QGroupBox::toggled, this, [&](bool on){
            reinterpret_cast<GraphicsWidget*>(this->parent())->setGraphicsItemsVisible(on);
        });
        layout.addWidget(groupBoxDrawingOption);

        QHBoxLayout *layoutRectColor = new QHBoxLayout;
        QLabel *labelRectColor = new QLabel("Rectangle Color :");
        QPushButton *pushButtonRectColor = new QPushButton("");
        pushButtonRectColor->setFixedSize(30, 20);
        pushButtonRectColor->setStyleSheet("QPushButton { background-color: rgba(20,72,126, 127); border: 1px solid lightgray; }");
        layoutRectColor->addWidget(labelRectColor);
        layoutRectColor->addWidget(pushButtonRectColor);
        connect(pushButtonRectColor, &QPushButton::clicked, this, [=](){
            auto color = QColorDialog::getColor(QColor(20,72,126,127), this, "Choose a color", QColorDialog::ColorDialogOption::ShowAlphaChannel);
            if(!color.isValid()) return;
            pushButtonRectColor->setStyleSheet("QPushButton {background-color:" + color.name() + "; border: 1px solid lightgray; }");
            reinterpret_cast<GraphicsWidget*>(this->parent())->setRectColor(color);
        });
        layoutDrawingOption->addLayout(layoutRectColor);

        QHBoxLayout *layoutEllipseColor = new QHBoxLayout;
        QLabel *labelEllipseColor = new QLabel("Ellipse Color :");
        QPushButton *pushButtonEllipseColor = new QPushButton("");
        pushButtonEllipseColor->setFixedSize(30, 20);
        pushButtonEllipseColor->setStyleSheet("QPushButton { background-color: rgba(104, 189, 69,127); border: 1px solid lightgray; }");
        layoutEllipseColor->addWidget(labelEllipseColor);
        layoutEllipseColor->addWidget(pushButtonEllipseColor);
        connect(pushButtonEllipseColor, &QPushButton::clicked, this, [=](){
            auto color = QColorDialog::getColor(QColor(104, 189, 69,127), this, "Choose a color", QColorDialog::ColorDialogOption::ShowAlphaChannel);
            if(!color.isValid()) return;
            pushButtonEllipseColor->setStyleSheet("QPushButton {background-color:" + color.name() + "; border: 1px solid lightgray; }");
            reinterpret_cast<GraphicsWidget*>(this->parent())->setEllipseColor(color);
        });
        layoutDrawingOption->addLayout(layoutEllipseColor);

        QHBoxLayout *layoutOutlineColor = new QHBoxLayout;
        QLabel *labelOutlineColor = new QLabel("Outline Color :");
        QPushButton *pushButtonOutlineColor = new QPushButton("");
        pushButtonOutlineColor->setFixedSize(30, 20);
        pushButtonOutlineColor->setStyleSheet("QPushButton { background-color: rgba(249,157,51,127); border: 1px solid lightgray; }");
        layoutOutlineColor->addWidget(labelOutlineColor);
        layoutOutlineColor->addWidget(pushButtonOutlineColor);
        connect(pushButtonOutlineColor, &QPushButton::clicked, this, [=](){
            auto color = QColorDialog::getColor(QColor(249,157,51,127), this, "Choose a color", QColorDialog::ColorDialogOption::ShowAlphaChannel);
            if(!color.isValid()) return;
            pushButtonOutlineColor->setStyleSheet("QPushButton {background-color:" + color.name() + "; border: 1px solid lightgray; }");
            reinterpret_cast<GraphicsWidget*>(this->parent())->setOutlineColor(color);
        });
        layoutDrawingOption->addLayout(layoutOutlineColor);

        QVBoxLayout *layoutCrosshair = new QVBoxLayout;
        QGroupBox *groupBoxCrosshair = new QGroupBox("Crosshair Options");
        groupBoxCrosshair->setLayout(layoutCrosshair);
        QHBoxLayout *layoutCrosshairWidth = new QHBoxLayout;
        QLabel *labelCrosshairWidth = new QLabel("Width :");
        QSpinBox *spinBoxCrosshairWidth = new QSpinBox;
        spinBoxCrosshairWidth->setRange(1,20);
        spinBoxCrosshairWidth->setValue(3);
        spinBoxCrosshairWidth->setFixedWidth(35);
        connect(spinBoxCrosshairWidth, QOverload<int>::of(&QSpinBox::valueChanged), this, [&](int val){
            auto widget = reinterpret_cast<GraphicsWidget*>(this->parent());
            crosshairWidth = val;
            widget->setCrossHair(widget->IsCrosshairEnabled(), crosshairColor, crosshairWidth);
        });
        layoutCrosshairWidth->addWidget(labelCrosshairWidth);
        layoutCrosshairWidth->addWidget(spinBoxCrosshairWidth);
        layoutCrosshair->addLayout(layoutCrosshairWidth);
        QHBoxLayout *layoutCrosshairColor = new QHBoxLayout;
        QLabel *labelCrosshairColor = new QLabel("Color :");
        QPushButton *pushButtonCrosshairColor = new QPushButton("");
        pushButtonCrosshairColor->setFixedSize(30,20);
        pushButtonCrosshairColor->setStyleSheet("QPushButton { background-color: rgba(104,189,69, 180); border: 1px solid lightgray; }");
        connect(pushButtonCrosshairColor, &QPushButton::clicked, this, [=](){
            auto color = QColorDialog::getColor(QColor(104,189,69, 180), this, "Choose a color", QColorDialog::ColorDialogOption::ShowAlphaChannel);
            if(!color.isValid()) return;
            crosshairColor = color;
            pushButtonCrosshairColor->setStyleSheet("QPushButton {background-color:" + color.name() + "; border: 1px solid lightgray; }");
            auto widget = reinterpret_cast<GraphicsWidget*>(this->parent());
            widget->setCrossHair(widget->IsCrosshairEnabled(), crosshairColor, crosshairWidth);
        });
        layoutCrosshairColor->addWidget(labelCrosshairColor);
        layoutCrosshairColor->addWidget(pushButtonCrosshairColor);
        layoutCrosshair->addLayout(layoutCrosshairColor);

        layout.addWidget(groupBoxCrosshair);

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

        QGroupBox *groupBoxCrossHair = new QGroupBox;
        QVBoxLayout *layoutCrossHair = new QVBoxLayout;
        groupBoxCrossHair->setLayout(layoutCrossHair);

        setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        setMinimumSize(250, 150);
    }
    QColor getCrosshairColor(){ return crosshairColor; }
    int getCrosshairWidth(){ return crosshairWidth; }
signals:
    void setBitShift(int value);

private:
    QVBoxLayout layout;
    QColor crosshairColor = QColor(104,189,69, 180);
    int crosshairWidth = 3;
};
}
#endif // GRAPHICSWIDGET_H
