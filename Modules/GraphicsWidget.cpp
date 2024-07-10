#include "GraphicsWidget.h"
#ifdef TIFF_ENABLED
#include "TiffReader.h"
#endif
#include "Processing/ImageTools.h"

Qylon::GraphicsWidget::GraphicsWidget(QWidget *parent)
    : settings(new GraphicsWidgetSettings(this))
    , view(new GraphicsView)
    , labelCoordinate(new QLabel)
    , labelPixelColor(new QLabel)
    , labelImageInformation(new QLabel)
    , lineEditPixelColor(new QLineEdit)
{
    setParent(parent);
    setWindowIcon(QIcon(":/Resources/Icon.png"));

    toolBar.setWindowTitle("Image Tools");
    toolBar.layout()->setSpacing(1);
    toolBar.setIconSize(QSize(24,24));

    statusBar.setSizeGripEnabled(false);
    statusBar.setStyleSheet("QStatusBar::item { border: none; }");

    setLayout(&layout);
    layout.setSpacing(0);
    layout.setContentsMargins(0,0,0,0);

    labelCoordinate->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    labelCoordinate->setText(" X 0\n Y 0");
    labelCoordinate->setFixedWidth(90);

    labelPixelColor->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    labelPixelColor->setText(" RGB\n [0,0,0]");
    labelPixelColor->setFixedWidth(90);

    labelImageInformation->setFixedWidth(170);

    lineEditPixelColor->setFixedWidth(24);
    lineEditPixelColor->setReadOnly(true);
    lineEditPixelColor->setFrame(false);

    settings->setBaseSize(500,500);

    connect(&fpsTimer, &QTimer::timeout, this, [&](){
        labelFPS.setText("FPS:" + QString::number(timerCnt));
        timerCnt=0;
    });
}

Qylon::GraphicsWidget::~GraphicsWidget(){
    delete scene;
}

void Qylon::GraphicsWidget::initialize(bool isVTK){
    scene = new GraphicsScene(isVTK);
    view->setScene(scene);

    QWidget *spacer = new QWidget;
    spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    toolBar.addWidget(spacer);

    QAction *actionLoad = new QAction(QIcon(":/Resources/Icon/icons8-image-file-add-48.png"),"", this);
    actionLoad->setToolTip("Load an image");
    toolBar.addAction(actionLoad);
    connect(actionLoad, &QAction::triggered, this, [=](){
        auto fileName = QFileDialog::getOpenFileName(this, "Load an image", QDir::currentPath(), "Image(*.png *.jpg *.jpeg *bmp *.tiff *.tif)");
        if(fileName.isEmpty()) return;

        if(fileName.contains(".tiff") || fileName.contains(".tif")){
#ifdef TIFF_ENABLED
            this->setImage(openTiff(fileName));
#endif
        }else this->setImage(QImage(fileName));
    });

    QAction *actionSave = new QAction(QIcon(":/Resources/Icon/icons8-save-as-48.png"), "", this);
    actionSave->setToolTip("Save the current image");
    toolBar.addAction(actionSave);
    connect(actionSave, &QAction::triggered, this, [=](){
        auto filePath = QFileDialog::getSaveFileName(this, "Save the current image", QDir::currentPath() + "/untitled.png", "Image(*.png *.jpg *.bmp *.tif *.tiff)");
        if(filePath.isEmpty()) return;

        if(this->currentImage.format() == QImage::Format_Grayscale16 && (filePath.contains("*.tif") || filePath.contains("*.tiff"))){
            return;
        }
        if(this->currentImage.save(filePath)){
            QMessageBox::information(this, "", "This current image was successfully saved.");
        }else{
            QMessageBox::critical(this, "", "Failed to save this current image.");
        }
    });


    QAction *actionGridLine = new QAction(QIcon(":/Resources/Icon/icons8-crosshair-48.png"), "", this);
    actionGridLine->setCheckable(true);
    actionGridLine->setToolTip("Crosshair");
    toolBar.addAction(actionGridLine);
    connect(actionGridLine, &QAction::toggled, this, [=](bool on){
        if(scene->Pixmap.pixmap().isNull()){
            actionGridLine->setChecked(false);
            return;
        }
        setCrossHair(on, settings->getCrosshairColor(), settings->getCrosshairWidth());
    });

    QAction *actionSettings = new QAction(QIcon(":/Resources/Icon/icons8-setting-48.png"), "", this);
    actionSettings->setToolTip("Settings");
    toolBar.addAction(actionSettings);
    connect(actionSettings, &QAction::triggered, this, [=](){
        settings->exec();
    });
    connect(settings, &GraphicsWidgetSettings::setBitShift, this, &GraphicsWidget::setBitShift);


    toolBar.addSeparator();

    QDoubleSpinBox *doubleSpinBoxRatio = new QDoubleSpinBox;
#if QT_VERSION >= QT_VERSION_CHECK(6,0,0)
    doubleSpinBoxRatio->setFixedWidth(70);
#else
    doubleSpinBoxRatio->setFixedWidth(60);
#endif
    doubleSpinBoxRatio->setStyleSheet("QDoubleSpinBox{border-radius:5px;color:white;background-color:#15487f;height:24px;}"
                                      "QDoubleSpinBox::up-button{width:0px; }"
                                      "QDoubleSpinBox::down-button{width:0px; }");
    doubleSpinBoxRatio->setReadOnly(true);
    doubleSpinBoxRatio->setFrame(false);
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

    QAction *actionZoomIn = new QAction(QIcon(":/Resources/Icon/icons8-zoom-in-48.png"), "");
    actionZoomIn->setToolTip("Zoom In");
    connect(actionZoomIn, &QAction::triggered, this, [=](){
        if(isVTK){
#ifdef PCL_ENABLED
            this->scene->VTKWidget->setScale(1.2);
#endif
        }else this->view->setScale((float)1.2);
    });
    toolBar.addAction(actionZoomIn);

    QAction *actionZoomOut = new QAction(QIcon(":/Resources/Icon/icons8-zoom-out-48.png"), "");
    actionZoomOut->setToolTip("Zoom Out");
    connect(actionZoomOut, &QAction::triggered, this, [=](){
        if(isVTK){
#ifdef PCL_ENABLED
            this->scene->VTKWidget->setScale(0.8);
#endif
        }else this->view->setScale((float)0.8);
    });
    toolBar.addAction(actionZoomOut);

    QAction *actionOriginal = new QAction(QIcon(":/Resources/Icon/icons8-one-to-one-48.png"), "");
    actionOriginal->setToolTip("100%");
    connect(actionOriginal, &QAction::triggered, this, [=](){
        if(isVTK){
#ifdef PCL_ENABLED
            this->scene->VTKWidget->resetScale();
#endif
        }else this->view->resetScale();
    });
    toolBar.addAction(actionOriginal);

    QAction *actionFit = new QAction(QIcon(":/Resources/Icon/icons8-separating-48.png"), "");
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
            // connect(buttonDown, &QPushButton::clicked, this, [=]() { this->scene->VTKWidget->seViewDown(); });
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
            this->labelCoordinate->setText(coord);

            // auto corr = (int)((r + g + b) / 3) > 150 ? 0 : 255;
            QString style =  QString("QLineEdit { background-color : rgb(") + QString::number(r) + ", " + QString::number(g) + ", " + QString::number(b) + QString("); }");
            this->lineEditPixelColor->setStyleSheet(style);

            QString color;
            if(currentImage.depth() == 16){
                if(currentImage.allGray()){
                    const uint16_t *line = reinterpret_cast<const uint16_t *>(currentImage.scanLine(rPos.y()));
                    uint16_t pixelValue = line[rPos.x()];
                    color = QString::number(pixelValue) +" (" + QString("0x%1").arg(pixelValue, 4, 16, QLatin1Char('0')).toUpper() + ")";
                }else{
                    qDebug() << "16bit color image setting is not set yet";
                }
            }else{
                if(currentImage.allGray()){
                    color = QString::number(currentImage.pixelColor(rPos.x(), rPos.y()).value());
                }else{
                    color = " RGB\n [" + QString::number(r) + "," + QString::number(g) + "," + QString::number(b) + "]";
                }
            }
            this->labelPixelColor->setText(color);
        });
        QLabel *lbl1 = new QLabel("|");
        lbl1->setStyleSheet("color:lightgray;");
        QLabel *lbl2 = new QLabel("|");
        lbl2->setStyleSheet("color:lightgray;");
        QLabel *lbl3 = new QLabel("|");
        lbl3->setStyleSheet("color:lightgray;");

        statusBar.addWidget(labelCoordinate);
        statusBar.addWidget(lbl1);
        statusBar.addWidget(lineEditPixelColor);
        statusBar.addWidget(labelPixelColor);
        statusBar.addWidget(lbl2);
        statusBar.addWidget(labelImageInformation);
        statusBar.addWidget(lbl3);

        statusBar.addWidget(&labelFPS);
    }

    // Putting Graphics View
    layout.addWidget(&toolBar);
    layout.addWidget(view);
    layout.addWidget(&statusBar);
}

void Qylon::GraphicsWidget::setToolBarEnable(bool on){
    toolBar.setHidden(!on);
}

void Qylon::GraphicsWidget::setGraphicsItemsVisible(bool on)
{
    for(int i=0; i<scene->items().size(); ++i){
        if(scene->items().at(i) == lineH || scene->items().at(i) == lineV) continue;
        if(&scene->Pixmap != scene->items().at(i)) scene->items().at(i)->setVisible(on);
    }
    graphicsItemVisible = on;
}

void Qylon::GraphicsWidget::clear()
{
    QMutexLocker locker(&mutex);
    QList<QGraphicsItem*> queue;
    try{
        for(auto &item : scene->items()){
            if(item != &scene->Pixmap){
                if(item == lineV || item == lineH) continue;
                scene->removeItem(item);
                delete item;
            }else{
                scene->Pixmap.setPixmap(QPixmap());
            }
        }
    }catch(std::exception &e){
        qDebug() << "clear function :" << e.what();
    }catch(...){
        qDebug() << "clear function :" << "Unknowned error occurred.";
    }
}

void Qylon::GraphicsWidget::reset()
{
    clear();
    scene->setSceneRect(0,0,0,0);
    setLogo(true);
}

void Qylon::GraphicsWidget::setLogo(bool on){
    view->setLogo(on);
}

void Qylon::GraphicsWidget::setCrossHair(bool on, QColor color, int width)
{
    crosshair = on;
    color = settings->getCrosshairColor();
    width = settings->getCrosshairWidth();

    if(on){
        if(!lineH){
            lineH = new QGraphicsLineItem;
        }
        if(!lineV){
            lineV = new QGraphicsLineItem;
        }
        QPen pen(color);
        pen.setWidth(width);
        lineH->setPen(pen);
        lineH->setLine(0, scene->sceneRect().height()/2, scene->sceneRect().width(), scene->sceneRect().height()/2);
        lineH->setZValue(10);
        lineV->setPen(pen);
        lineV->setLine(scene->sceneRect().width()/2,0,scene->sceneRect().width()/2, scene->sceneRect().height());
        lineV->setZValue(10);
        if(!scene->items().contains(lineH)) scene->addItem(lineH);
        if(!scene->items().contains(lineV)) scene->addItem(lineV);
    }else{
        for(int i=0; i< toolBar.actions().size(); ++i){
            if(toolBar.actions().at(i)->toolTip() == "Crosshair"){
                toolBar.actions().at(i)->setChecked(false);
            }
        }
        delete lineH;
        delete lineV;
        lineH = nullptr;
        lineV = nullptr;

    }
}

void Qylon::GraphicsWidget::setImage(const QImage image){
    QMutexLocker locker(&mutex);

    ++timerCnt;
    currentImage = image;

    if(bitShift !=0 && image.format() == QImage::Format_Grayscale16){
        scene->Pixmap.setPixmap(QPixmap::fromImage(shiftImage(image, bitShift)));
    }else{
        scene->Pixmap.setPixmap(QPixmap::fromImage(image));
    }
    labelImageInformation->setText(QString::number(image.depth()) + "-bit, " +
                                   QString((image.isGrayscale()) ? "Mono" : "Color") +
                                   ", " +
                                   QString::number(image.width()) + "x" +
                                   QString::number(image.height()));
    scene->setSceneRect(0, 0, image.width(), image.height());
    if(lineH) lineH->setLine(0, scene->sceneRect().height()/2, scene->sceneRect().width(), scene->sceneRect().height()/2);
    if(lineV) lineV->setLine(scene->sceneRect().width()/2,0,scene->sceneRect().width()/2, scene->sceneRect().height());

    view->setFit(view->isFit());
    view->setLogo(false);
}

void Qylon::GraphicsWidget::updateImage()
{
    setImage(currentImage);
}

bool Qylon::GraphicsWidget::drawGraphicsItem(QGraphicsItem *item)
{
    QMutexLocker locker(&mutex);
    try{
        if(!item) return false;
        if(QGraphicsRectItem* rect = dynamic_cast<QGraphicsRectItem*>(item)){
            QBrush brush(rectangleColor);
            rect->setBrush(brush);
            rect->setPen(Qt::NoPen);
        }else if(QGraphicsEllipseItem *ellipse = dynamic_cast<QGraphicsEllipseItem*>(item)){
            QBrush brush(ellipseColor);
            ellipse->setBrush(brush);
            ellipse->setPen(Qt::NoPen);
        }else if(QGraphicsLineItem *line = dynamic_cast<QGraphicsLineItem*>(item)){
            QPen pen(outlineColor);
            pen.setWidth(2);
            line->setPen(pen);
        }else{
            qDebug() <<"Else case occurred";
            return false;
        }
        item->setVisible(graphicsItemVisible);
        scene->addItem(item);
    }catch(std::exception &e){
        qDebug() << "DrawGraphicsItem function :" << e.what();
        return false;
    }catch(...){
        qDebug() << "DrawGraphicsItem function :" << "Unknowned error occurred.";
        return false;
    }
    return true;
}

void Qylon::GraphicsWidget::setFit(bool on){
    view->setFit(on);
    emit updateWidget();
}

void Qylon::GraphicsWidget::setFPSEnable(bool on){
    if(on){
        fpsTimer.start(1000);
    }else{
        fpsTimer.stop();
    }
}

void Qylon::GraphicsWidget::removeAllGraphicsItem(){
    clear();
}
#ifdef PCL_ENABLED
void Qylon::GraphicsWidget::setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudData, QString pointCloudName){
    scene->VTKWidget->setPointCloud(cloudData, pointCloudName);
    view->viewport()->update();
}
#endif
