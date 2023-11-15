#ifndef GRAPHICSWIDGET_H
#define GRAPHICSWIDGET_H
#include <QWidget>

#include "GraphicsView.h"
#include "GraphicsScene.h"
#include <QElapsedTimer>
namespace Qylon{
class GraphicsWidget : public QWidget{
    Q_OBJECT
public:
    GraphicsWidget(){
        setLayout(&layout);
        layout.setSpacing(0);
        layout.setMargin(0);
        view = new GraphicsView;
        view->setDragMode(QGraphicsView::ScrollHandDrag);
        labelCoordinate.setAlignment(Qt::AlignRight);
    }
    void initialize(bool isVTK=false){
        scene = new GraphicsScene(isVTK);
        view->setScene(scene);
        QHBoxLayout *buttonLayout = new QHBoxLayout;
        {
            buttonLayout->addWidget(&labelRatio);

            QPushButton *buttonZoomIn = new QPushButton("+");
            buttonZoomIn->setFixedWidth(30);
            connect(buttonZoomIn, &QPushButton::clicked, this, [=](){
                if(isVTK){
#ifdef PCL_ENABLED
                    this->scene->VTKWidget->setScale(1.2);
#endif
                }else this->view->setScale(1.2);
            });
            buttonLayout->addWidget(buttonZoomIn);

            QPushButton *buttonZoomOut = new QPushButton("-");
            buttonZoomOut->setFixedWidth(30);
            connect(buttonZoomOut, &QPushButton::clicked, this, [=](){
                if(isVTK){
#ifdef PCL_ENABLED
                    this->scene->VTKWidget->setScale(0.8);
#endif
                }else this->view->setScale(0.8);
            });
            buttonLayout->addWidget(buttonZoomOut);

//            QPushButton *buttonOriginal = new QPushButton("↺");
            QPushButton *buttonOriginal = new QPushButton("R");
            buttonOriginal->setFixedWidth(30);
            connect(buttonOriginal, &QPushButton::clicked, this, [=](){
                if(isVTK){
#ifdef PCL_ENABLED
                    this->scene->VTKWidget->resetScale();
#endif
                }else this->view->resetScale();
            });
            buttonLayout->addWidget(buttonOriginal);

//            QPushButton *buttonFit = new QPushButton("↔");
            QPushButton *buttonFit = new QPushButton("F");
            buttonFit->setFixedWidth(30);
            buttonFit->setCheckable(true);
            buttonFit->setChecked(false);
            connect(buttonFit, &QPushButton::toggled, this, [=](bool on){
                this->view->setFit(on);

                buttonFit->setChecked(on);
                buttonZoomIn->setEnabled(!on);
                buttonZoomOut->setEnabled(!on);
                buttonOriginal->setEnabled(!on);
            });
            buttonLayout->addWidget(buttonFit);
            connect(this, &GraphicsWidget::updateWidget, this, [=](){
                emit buttonFit->toggled(true);

            });
        }
#ifdef PCL_ENABLED
        if(isVTK){
            QPushButton *buttonUp = new QPushButton("↑");
            {
                buttonUp->setFixedWidth(30);
                connect(buttonUp, &QPushButton::clicked, this, [=](){
                    this->scene->VTKWidget->setViewUp();
                });
                buttonLayout->addWidget(buttonUp);
            }
            QPushButton *buttonDown = new QPushButton("↓");
            {
                buttonDown->setFixedWidth(30);
                connect(buttonDown, &QPushButton::clicked, this, [=](){
                    this->scene->VTKWidget->seViewDown();
                });
                buttonLayout->addWidget(buttonDown);
            }
            QPushButton *buttonLeft = new QPushButton("←");
            {
                buttonLeft->setFixedWidth(30);
                connect(buttonLeft, &QPushButton::clicked, this, [=](){
                    this->scene->VTKWidget->setViewLeft();
                });
                buttonLayout->addWidget(buttonLeft);
            }
            QPushButton *buttonRight = new QPushButton("→");
            {
                buttonRight->setFixedWidth(30);
                connect(buttonRight, &QPushButton::clicked, this, [=](){
                    this->scene->VTKWidget->setViewRight();
                });
                buttonLayout->addWidget(buttonRight);
            }

            /*
//            QLabel *lbl = new QLabel;
//            connect(this->scene->VTKWidget, &GraphicsVTKWidget::currentCameraPosition, this, [=](QString text){
//                lbl->setText(text);
//            });
            QHBoxLayout *spinLayout = new QHBoxLayout;
            QDoubleSpinBox *spinPosX = new QDoubleSpinBox;
            spinPosX->setRange(-10000, 10000);
            spinPosX->setValue(0);
            QDoubleSpinBox *spinPosY = new QDoubleSpinBox;
            spinPosY->setRange(-10000, 10000);
            spinPosY->setValue(0);
            QDoubleSpinBox *spinPosZ = new QDoubleSpinBox;
            spinPosZ->setRange(-10000, 10000);
            spinPosZ->setValue(-500);
            QDoubleSpinBox *spinFocalX = new QDoubleSpinBox;
            spinFocalX->setRange(-10000, 10000);
            spinFocalX->setValue(0);
            QDoubleSpinBox *spinFocalY = new QDoubleSpinBox;
            spinFocalY->setRange(-10000, 10000);
            spinFocalY->setValue(0);
            QDoubleSpinBox *spinFocalZ = new QDoubleSpinBox;
            spinFocalZ->setRange(-10000, 10000);
            spinFocalZ->setValue(0);
            QDoubleSpinBox *spinViewX = new QDoubleSpinBox;
            QDoubleSpinBox *spinViewY = new QDoubleSpinBox;
            QDoubleSpinBox *spinViewZ = new QDoubleSpinBox;

            spinViewX->setRange(-1.0, 1.0);
            spinViewX->setSingleStep(0.05);
            spinViewX->setValue(0);
            spinViewY->setRange(-1.0, 1.0);
            spinViewY->setSingleStep(0.05);
            spinViewY->setValue(-1);
            spinViewZ->setRange(-1.0, 1.0);
            spinViewZ->setSingleStep(0.05);
            spinViewZ->setValue(0);

            connect(spinPosX,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=](double value){
                this->scene->VTKWidget->viewer->setCameraPosition(spinPosX->value(), spinPosY->value(), spinPosZ->value(),
                                                        spinFocalX->value(), spinFocalY->value(), spinFocalZ->value(),
                                                          spinViewX->value(), spinViewY->value(), spinViewZ->value());
            });
            connect(spinPosY,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=](double value){
                this->scene->VTKWidget->viewer->setCameraPosition(spinPosX->value(), spinPosY->value(), spinPosZ->value(),
                                                                  spinFocalX->value(), spinFocalY->value(), spinFocalZ->value(),
                                                                  spinViewX->value(), spinViewY->value(), spinViewZ->value());
            });
            connect(spinPosZ,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=](double value){
                this->scene->VTKWidget->viewer->setCameraPosition(spinPosX->value(), spinPosY->value(), spinPosZ->value(),
                                                                  spinFocalX->value(), spinFocalY->value(), spinFocalZ->value(),
                                                                  spinViewX->value(), spinViewY->value(), spinViewZ->value());
            });
            connect(spinFocalX,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=](double value){
                this->scene->VTKWidget->viewer->setCameraPosition(spinPosX->value(), spinPosY->value(), spinPosZ->value(),
                                                                  spinFocalX->value(), spinFocalY->value(), spinFocalZ->value(),
                                                                  spinViewX->value(), spinViewY->value(), spinViewZ->value());
            });
            connect(spinFocalY,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=](double value){
                this->scene->VTKWidget->viewer->setCameraPosition(spinPosX->value(), spinPosY->value(), spinPosZ->value(),
                                                                  spinFocalX->value(), spinFocalY->value(), spinFocalZ->value(),
                                                                  spinViewX->value(), spinViewY->value(), spinViewZ->value());
            });
            connect(spinFocalZ,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=](double value){
                this->scene->VTKWidget->viewer->setCameraPosition(spinPosX->value(), spinPosY->value(), spinPosZ->value(),
                                                                  spinFocalX->value(), spinFocalY->value(), spinFocalZ->value(),
                                                                  spinViewX->value(), spinViewY->value(), spinViewZ->value());
            });
            connect(spinViewX,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=](double value){
                this->scene->VTKWidget->viewer->setCameraPosition(spinPosX->value(), spinPosY->value(), spinPosZ->value(),
                                                                  spinFocalX->value(), spinFocalY->value(), spinFocalZ->value(),
                                                                  spinViewX->value(), spinViewY->value(), spinViewZ->value());
            });
            connect(spinViewY,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=](double value){
                this->scene->VTKWidget->viewer->setCameraPosition(spinPosX->value(), spinPosY->value(), spinPosZ->value(),
                                                                  spinFocalX->value(), spinFocalY->value(), spinFocalZ->value(),
                                                                  spinViewX->value(), spinViewY->value(), spinViewZ->value());
            });
            connect(spinViewZ,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=](double value){
                this->scene->VTKWidget->viewer->setCameraPosition(spinPosX->value(), spinPosY->value(), spinPosZ->value(),
                                                                  spinFocalX->value(), spinFocalY->value(), spinFocalZ->value(),
                                                                  spinViewX->value(), spinViewY->value(), spinViewZ->value());
            });

            spinLayout->addWidget(spinPosX);
            spinLayout->addWidget(spinPosY);
            spinLayout->addWidget(spinPosZ);
            spinLayout->addWidget(spinFocalX);
            spinLayout->addWidget(spinFocalY);
            spinLayout->addWidget(spinFocalZ);
            spinLayout->addWidget(spinViewX);
            spinLayout->addWidget(spinViewY);
            spinLayout->addWidget(spinViewZ);

            layout.addLayout(spinLayout);
*/
        }
#endif
        layout.addLayout(buttonLayout);

        // Putting Graphics View
        layout.addWidget(view);

        // Setting a status bar
        if(!isVTK){
            connect(scene, &GraphicsScene::currentPos, this, [=](QPointF point){
                auto rPos = point.toPoint();
                int r, g, b = 0;
                this->scene->Pixmap.pixmap().toImage().pixelColor(rPos.x(), rPos.y()).getRgb(&r, &g, &b);
                QString coord = "(X:" + QString::number(rPos.x()) + " Y:" + QString::number(rPos.y()) + ")";
                QString color = "(R:" + QString::number(r) + " G:" + QString::number(g) + " B:" + QString::number(b) + ")";

                this->labelCoordinate.setText(coord + " " + color);

                auto corr = (int)((r+g+b)/3) > 150 ? 0 : 255;
                QString style = QString("QLabel { background-color : rgb(") + QString::number(r) + ", " + QString::number(g) + ", " + QString::number(b) + QString("); color : rgb(") +
                        QString::number(corr) + QString(", ") + QString::number(corr) + QString(", ") + QString::number(corr) + QString("); }");

                this->labelCoordinate.setStyleSheet(style);
            });
            connect(view, &GraphicsView::currentRatio, this, [=](float ratio){
                this->labelRatio.setText("Ratio : " + QString::number(ratio*100) + "%");
            });
            QHBoxLayout *pixelInfoLayout = new QHBoxLayout;
            pixelInfoLayout->addWidget(&labelCoordinate);
            layout.addLayout(pixelInfoLayout);
        }
        setImage(QImage(":/Qylon/Resources/Logo.png"));
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
        QElapsedTimer time;
        qDebug() << "Recieved";
        time.start();
        scene->VTKWidget->setPointCloud(cloudData, pointCloudName);
        view->viewport()->update();
        qDebug() << time.elapsed();
    }
#endif

private:
    GraphicsView *view;
    GraphicsScene *scene;
    QLabel labelCoordinate = QLabel("(X:0 Y:0)");
    QLabel labelRatio = QLabel("Ratio : 100%");

    QVBoxLayout layout;
};

}
#endif // GRAPHICSWIDGET_H
