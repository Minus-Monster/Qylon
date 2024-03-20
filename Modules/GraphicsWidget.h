#ifndef GRAPHICSWIDGET_H
#define GRAPHICSWIDGET_H
#include <QWidget>
#include <QSpacerItem>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QToolBar>

#include "GraphicsView.h"
#include "GraphicsScene.h"
namespace Qylon{
class GraphicsWidget : public QWidget{
    Q_OBJECT
public:
    GraphicsWidget(QWidget *parent = nullptr){
        setParent(parent);
        setWindowIcon(QIcon(":/Resources/Icon.png"));

        toolBar.setWindowTitle("Image Tools");
        toolBar.layout()->setSpacing(1);
        toolBar.layout()->setContentsMargins(0,0,0,0);
        setLayout(&layout);
        layout.setSpacing(0);
        layout.setMargin(0);

        view = new GraphicsView;
        view->setDragMode(QGraphicsView::ScrollHandDrag);
        labelCoordinate.setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        labelCoordinate.setText(" X 0\n Y 0");
        labelCoordinate.setFixedWidth(60);
        labelPixelColor.setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        labelPixelColor.setText(" RGB\n [0,0,0]");
        labelPixelColor.setFixedWidth(98);
        lineEditPixelColor.setFixedWidth(24);
        lineEditPixelColor.setReadOnly(true);
    }
    void initialize(bool isVTK=false){
        scene = new GraphicsScene(isVTK);
        view->setScene(scene);

        QWidget *spacer = new QWidget;
        toolBar.addWidget(spacer);

        QPushButton *buttonGridLine = new QPushButton(QIcon(":/Resources/Icon/icons8-crosshair-48.png"), "");
        buttonGridLine->setFixedWidth(30);
        buttonGridLine->setFlat(true);
        buttonGridLine->setCheckable(true);
        buttonGridLine->setToolTip("Crosshair");
        toolBar.addWidget(buttonGridLine);
        connect(buttonGridLine, &QPushButton::clicked, this, [=](bool on){
            this->view->setCrossHair(on);
        });


        toolBar.addSeparator();
        spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

        QDoubleSpinBox *doubleSpinBoxRatio = new QDoubleSpinBox;
        {
            doubleSpinBoxRatio->setRange(0, 9999.9);
            doubleSpinBoxRatio->setValue(100);
            doubleSpinBoxRatio->setSuffix(" %");
            doubleSpinBoxRatio->setFixedWidth(75);
            doubleSpinBoxRatio->setDecimals(1);
            doubleSpinBoxRatio->setSingleStep(10);
            doubleSpinBoxRatio->setAlignment(Qt::AlignRight);
            doubleSpinBoxRatio->setToolTip("Scale");
            connect(doubleSpinBoxRatio, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=](){
                this->view->setRatio(doubleSpinBoxRatio->value()/100);
            });
            connect(view, &GraphicsView::currentRatio, this, [=](float ratio) {
                doubleSpinBoxRatio->setValue(ratio * 100);
            });
        }
        QPushButton *buttonZoomIn = new QPushButton(QIcon(":/Resources/Icon/zoom-in.png"), "");
        buttonZoomIn->setFixedWidth(30);
        buttonZoomIn->setFlat(true);
        buttonZoomIn->setToolTip("Zoom In");
        connect(buttonZoomIn, &QPushButton::clicked, this, [=](){
        if(isVTK){
#ifdef PCL_ENABLED
                this->scene->VTKWidget->setScale(1.2);
#endif
            }else this->view->setScale(1.2);
        });
        toolBar.addWidget(buttonZoomIn);

        QPushButton *buttonZoomOut = new QPushButton(QIcon(":/Resources/Icon/zoom-out.png"), "");
        buttonZoomOut->setFixedWidth(30);
        buttonZoomOut->setFlat(true);
        buttonZoomOut->setToolTip("Zoom Out");
        connect(buttonZoomOut, &QPushButton::clicked, this, [=](){
            if(isVTK){
#ifdef PCL_ENABLED
                this->scene->VTKWidget->setScale(0.8);
#endif
            }else this->view->setScale(0.8);
        });
        toolBar.addWidget(buttonZoomOut);

        QPushButton *buttonOriginal = new QPushButton(QIcon(":/Resources/Icon/original.png"), "");
        buttonOriginal->setFixedWidth(30);
        buttonOriginal->setFlat(true);
        buttonOriginal->setToolTip("100%");
        connect(buttonOriginal, &QPushButton::clicked, this, [=](){
            if(isVTK){
#ifdef PCL_ENABLED
                this->scene->VTKWidget->resetScale();
#endif
            }else this->view->resetScale();
        });
        toolBar.addWidget(buttonOriginal);

        QPushButton *buttonFit = new QPushButton(QIcon(":/Resources/Icon/fit.png"), "");
        buttonFit->setFlat(true);
        buttonFit->setFixedWidth(30);
        buttonFit->setCheckable(true);
        buttonFit->setChecked(false);
        buttonFit->setToolTip("Fit");
        connect(buttonFit, &QPushButton::toggled, this, [=](bool on){
            if(on) currentRatioValue = doubleSpinBoxRatio->value();
            this->view->setFit(on);

            buttonFit->setChecked(on);
            buttonZoomIn->setEnabled(!on);
            buttonZoomOut->setEnabled(!on);
            buttonOriginal->setEnabled(!on);
            doubleSpinBoxRatio->setEnabled(!on);
            if(!on) doubleSpinBoxRatio->setValue(currentRatioValue);
        });
        toolBar.addWidget(buttonFit);
        connect(this, &GraphicsWidget::updateWidget, this, [=](){
            emit buttonFit->toggled(true);
        });
        toolBar.addSeparator();
        toolBar.addWidget(doubleSpinBoxRatio);

#ifdef PCL_ENABLED
        if(isVTK){
            QPushButton *buttonUp = new QPushButton("↑");
            {
                buttonUp->setFixedWidth(30);
                connect(buttonUp, &QPushButton::clicked, this, [=](){
                    this->scene->VTKWidget->setViewUp();
                });
                toolBar.addWidget(buttonUp);
            }
            QPushButton *buttonDown = new QPushButton("↓");
            {
                buttonDown->setFixedWidth(30);
                connect(buttonDown, &QPushButton::clicked, this, [=]() { this->scene->VTKWidget->seViewDown(); });
                toolBar.addWidget(buttonDown);
            }
            QPushButton *buttonLeft = new QPushButton("←");
            {
                buttonLeft->setFixedWidth(30);
                connect(buttonLeft, &QPushButton::clicked, this, [=]() { this->scene->VTKWidget->setViewLeft(); });
                toolBar.addWidget(buttonLeft);
            }
            QPushButton *buttonRight = new QPushButton("→");
            {
                buttonRight->setFixedWidth(30);
                connect(buttonRight, &QPushButton::clicked, this, [=]() { this->scene->VTKWidget->setViewRight(); });
                toolBar.addWidget(buttonRight);
            }
        }
#endif
        if (!isVTK) {
            connect(scene, &GraphicsScene::currentPos, this, [=](QPointF point) {
                auto rPos = point.toPoint();
                int r, g, b = 0;
                this->scene->Pixmap.pixmap().toImage().pixelColor(rPos.x(), rPos.y()).getRgb(&r, &g, &b);
                QString coord = " X " + QString::number(rPos.x()) +"\n Y " + QString::number(rPos.y());
                QString color = " RGB\n [" + QString::number(r) +"," + QString::number(g) + "," + QString::number(b) + "]";

                this->labelCoordinate.setText(coord);

                auto corr = (int)((r + g + b) / 3) > 150 ? 0 : 255;
                QString style =  QString("QLineEdit { background-color : rgb(") + QString::number(r) + ", " + QString::number(g) + ", " + QString::number(b) + QString("); }");

                this->labelPixelColor.setText(color);
                this->lineEditPixelColor.setStyleSheet(style);
            });
            toolBar.addSeparator();
            toolBar.addWidget(&labelCoordinate);
            toolBar.addWidget(&lineEditPixelColor);
            toolBar.addWidget(&labelPixelColor);
        }

        // Putting Graphics View
        layout.addWidget(&toolBar);
        layout.addWidget(view);
    }
    void setToolBarEnable(bool on){
        toolBar.setHidden(!on);
    }

signals:
    void updateWidget();

public slots:
    void setImage(const QImage image){
        scene->Pixmap.setPixmap(QPixmap::fromImage(image));
        scene->setSceneRect(0, 0, image.width(), image.height());
    }
    void setFit(bool on){
        view->setFit(on);
        emit updateWidget();
    }
#ifdef PCL_ENABLED
    void setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudData, QString pointCloudName){
        scene->VTKWidget->setPointCloud(cloudData, pointCloudName);
        view->viewport()->update();
    }
#endif

private:
    GraphicsView *view;
    GraphicsScene *scene;
    QLabel labelCoordinate;
    QLabel labelPixelColor;
    QLineEdit lineEditPixelColor;
    QToolBar toolBar;
    double currentRatioValue = 100.;

    QVBoxLayout layout;
};

}
#endif // GRAPHICSWIDGET_H
