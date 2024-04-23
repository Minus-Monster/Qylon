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
#ifdef TIFF_ENABLED
#include "TiffReader.h"
#endif
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
        toolBar.setIconSize(QSize(18,18));

        statusBar.setSizeGripEnabled(false);

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
    ~GraphicsWidget(){
        delete this;
    }
    void initialize(bool isVTK=false){
        scene = new GraphicsScene(isVTK);
        view->setScene(scene);

        QWidget *spacer = new QWidget;
        spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        toolBar.addWidget(spacer);

        QAction *actionLoad = new QAction(QIcon(":/Resources/Icon/icons8-image-file-add-48.png"),"");
        actionLoad->setToolTip("Load an image");
        toolBar.addAction(actionLoad);
        connect(actionLoad, &QAction::triggered, this, [=](){
            auto fileName = QFileDialog::getOpenFileName(this, "Load an image", QDir::currentPath(), "Image(*.png *.jpg *.jpeg *bmp *.tiff *.tif)");
            if(fileName.isEmpty()) return;

            if(fileName.contains(".tiff") || fileName.contains(".tif")){
                qDebug() << "This image is needed tiff converting";
#ifdef TIFF_ENABLED
                this->setImage(openTiff(fileName));
#endif
            }else this->setImage(QImage(fileName));
        });

        QAction *actionSave = new QAction(QIcon(":/Resources/Icon/icons8-save-as-48.png"), "");
        actionSave->setToolTip("Save the current image");
        toolBar.addAction(actionSave);
        connect(actionSave, &QAction::triggered, this, [=](){
            auto filePath = QFileDialog::getSaveFileName(this, "Save the current image", QDir::currentPath() + "/untitled.png", "Image(*.png *.jpg *.bmp *.tif *.tiff)");
            if(filePath.isEmpty()) return;

            if(this->currentImage.save(filePath)){
                QMessageBox::information(this, "", "This current image was successfully saved.");
            }else{
                QMessageBox::critical(this, "", "Failed to save this current image.");
            }
        });


        QAction *actionGridLine = new QAction(QIcon(":/Resources/Icon/icons8-crosshair-48.png"), "");
        actionGridLine->setCheckable(true);
        actionGridLine->setToolTip("Crosshair");
        toolBar.addAction(actionGridLine);
        connect(actionGridLine, &QAction::toggled, this, [=](bool on){
            this->view->setCrossHair(on);
        });

        toolBar.addSeparator();

        QDoubleSpinBox *doubleSpinBoxRatio = new QDoubleSpinBox;
        doubleSpinBoxRatio->setRange(0, 9999.9);
        doubleSpinBoxRatio->setValue(100);
        doubleSpinBoxRatio->setSuffix(" %");
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

        QAction *actionZoomIn = new QAction(QIcon(":/Resources/Icon/zoom-in.png"), "");
        actionZoomIn->setToolTip("Zoom In");
        connect(actionZoomIn, &QAction::triggered, this, [=](){
            if(isVTK){
#ifdef PCL_ENABLED
                this->scene->VTKWidget->setScale(1.2);
#endif
            }else this->view->setScale((float)1.2);
        });
        toolBar.addAction(actionZoomIn);

        QAction *actionZoomOut = new QAction(QIcon(":/Resources/Icon/zoom-out.png"), "");
        actionZoomOut->setToolTip("Zoom Out");
        connect(actionZoomOut, &QAction::triggered, this, [=](){
            if(isVTK){
#ifdef PCL_ENABLED
                this->scene->VTKWidget->setScale(0.8);
#endif
            }else this->view->setScale((float)0.8);
        });
        toolBar.addAction(actionZoomOut);

        QAction *actionOriginal = new QAction(QIcon(":/Resources/Icon/original.png"), "");
        actionOriginal->setToolTip("100%");
        connect(actionOriginal, &QAction::triggered, this, [=](){
            if(isVTK){
#ifdef PCL_ENABLED
                this->scene->VTKWidget->resetScale();
#endif
            }else this->view->resetScale();
        });
        toolBar.addAction(actionOriginal);

        QAction *actionFit = new QAction(QIcon(":/Resources/Icon/fit.png"), "");
        actionFit->setCheckable(true);
        actionFit->setChecked(false);
        actionFit->setToolTip("Fit");
        connect(actionFit, &QAction::triggered, this, [=](bool on){
            if(on) currentRatioValue = doubleSpinBoxRatio->value();
            this->view->setFit(on);

            actionFit->setChecked(on);
            actionZoomIn->setEnabled(!on);
            actionZoomOut->setEnabled(!on);
            actionOriginal->setEnabled(!on);
            doubleSpinBoxRatio->setEnabled(!on);
            if(!on) doubleSpinBoxRatio->setValue(currentRatioValue);
        });
        toolBar.addAction(actionFit);
        connect(this, &GraphicsWidget::updateWidget, this, [=](){
            emit actionFit->toggled(true);
        });
        toolBar.addWidget(doubleSpinBoxRatio);

#ifdef PCL_ENABLED
        if(isVTK){
            QPushButton *buttonUp = new QPushButton("Up");
            {
                connect(buttonUp, &QPushButton::clicked, this, [=](){
                    this->scene->VTKWidget->setViewUp();
                });
                toolBar.addWidget(buttonUp);
            }
            QPushButton *buttonDown = new QPushButton("Down");
            {
                connect(buttonDown, &QPushButton::clicked, this, [=]() { this->scene->VTKWidget->seViewDown(); });
                toolBar.addWidget(buttonDown);
            }
            QPushButton *buttonLeft = new QPushButton("Left");
            {
                connect(buttonLeft, &QPushButton::clicked, this, [=]() { this->scene->VTKWidget->setViewLeft(); });
                toolBar.addWidget(buttonLeft);
            }
            QPushButton *buttonRight = new QPushButton("Right");
            {
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
                this->labelCoordinate.setText(coord);


                // auto corr = (int)((r + g + b) / 3) > 150 ? 0 : 255;
                QString style =  QString("QLineEdit { background-color : rgb(") + QString::number(r) + ", " + QString::number(g) + ", " + QString::number(b) + QString("); }");
                this->lineEditPixelColor.setStyleSheet(style);

                QString color;
                if(currentImage.depth() == 16){
                    if(currentImage.allGray()){
                        color = QString::number(currentImage.pixelColor(rPos.x(), rPos.y()).value());
                        const int bytes = (currentImage.depth() + 7) / 8;
                        assert(bytes > 0);
                        const QByteArray data((const char*)(currentImage.bits() +
                                                            rPos.y() *currentImage.bytesPerLine() +
                                                            (rPos.x() * currentImage.depth() + 7)/8),
                                              bytes);
                        color = QString::number(data.toHex().toLong(nullptr, 16)) +" (" + "0x" + data.toHex().toUpper()+ ")";
                    }else{
                        qDebug() << "16bit color image setting is not set yet";
                    }
                }else{
                    if(currentImage.allGray()){
                        color = QString::number(currentImage.pixelColor(rPos.x(), rPos.y()).value());
                    }else{
                        color = " RGB\n [" + QString::number(r) +"," + QString::number(g) + "," + QString::number(b) + "]";
                    }
                }
                this->labelPixelColor.setText(color);
            });
            statusBar.addWidget(&labelCoordinate);
            statusBar.addWidget(&lineEditPixelColor);
            statusBar.addWidget(&labelPixelColor);
            statusBar.addWidget(&labelImageInformation);
        }

        // Putting Graphics View
        layout.addWidget(&toolBar);
        layout.addWidget(view);
        layout.addWidget(&statusBar);
    }
    void setToolBarEnable(bool on){
        toolBar.setHidden(!on);
    }

signals:
    void updateWidget();

public slots:
    void setImage(const QImage image){
        currentImage = image;
        labelImageInformation.setText(QString::number(image.depth()) + "-bit, " +
                                      QString((image.isGrayscale()) ? "Mono" : "Color") +
                                      ", " +
                                      QString::number(image.width()) + "x" +
                                      QString::number(image.height()));
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
    QLabel labelImageInformation;
    QLineEdit lineEditPixelColor;
    QToolBar toolBar;
    QStatusBar statusBar;
    QImage currentImage;
    double currentRatioValue = 100.;

    QVBoxLayout layout;
};

}
#endif // GRAPHICSWIDGET_H
