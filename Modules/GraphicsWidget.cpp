#include "GraphicsWidget.h"
#ifdef TIFF_ENABLED
#include "TiffReader.h"
#endif

Qylon::GraphicsWidget::GraphicsWidget(QWidget *parent){
    setParent(parent);
    setWindowIcon(QIcon(":/Resources/Icon.png"));

    toolBar.setWindowTitle("Image Tools");
    toolBar.layout()->setSpacing(1);
    toolBar.setIconSize(QSize(18,18));

    statusBar.setSizeGripEnabled(false);
    statusBar.setStyleSheet("QStatusBar::item { border: none; }");

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
    lineEditPixelColor.setFrame(false);

    connect(&fpsTimer, &QTimer::timeout, this, [&](){
        labelFPS.setText("FPS:" + QString::number(timerCnt));
        timerCnt=0;
    });
}

Qylon::GraphicsWidget::~GraphicsWidget(){
    deleteLater();
}

void Qylon::GraphicsWidget::initialize(bool isVTK){
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

        if(this->currentImage.format() == QImage::Format_Grayscale16 && (filePath.contains("*.tif") || filePath.contains("*.tiff"))){
            qDebug() << "16 bit image." << saveQImageToTiff16(this->currentImage, filePath);
            return;
        }
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

    QAction *actionSettings = new QAction(QIcon(":/Resources/Icon/icons8-file-settings-48.png"), "");
    actionSettings->setCheckable(false);
    actionSettings->setToolTip("Settings");
    toolBar.addAction(actionSettings);
    connect(actionSettings, &QAction::toggled, this, [=](){

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
                    color = " RGB\n [" + QString::number(r) +"," + QString::number(g) + "," + QString::number(b) + "]";
                }
            }
            this->labelPixelColor.setText(color);
        });
        statusBar.addWidget(&labelCoordinate);
        statusBar.addWidget(&lineEditPixelColor);
        statusBar.addWidget(&labelPixelColor);
        statusBar.addWidget(&labelImageInformation);
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

void Qylon::GraphicsWidget::setImage(const QImage image){
    ++timerCnt;
    currentImage = image;
    labelImageInformation.setText(QString::number(image.depth()) + "-bit, " +
                                  QString((image.isGrayscale()) ? "Mono" : "Color") +
                                  ", " +
                                  QString::number(image.width()) + "x" +
                                  QString::number(image.height()));
    scene->Pixmap.setPixmap(QPixmap::fromImage(image));
    scene->setSceneRect(0, 0, image.width(), image.height());

    emit scene->currentPos(scene->getCurrentMousePoint());
    view->setLogo(false);
}

void Qylon::GraphicsWidget::setFit(bool on){
    view->setFit(on);
    emit updateWidget();
}

void Qylon::GraphicsWidget::setTimerEnable(bool on){
    if(on){
        fpsTimer.start(1000);
    }else{
        fpsTimer.stop();
    }
}
#ifdef PCL_ENABLED
void Qylon::GraphicsWidget::setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudData, QString pointCloudName){
    scene->VTKWidget->setPointCloud(cloudData, pointCloudName);
    view->viewport()->update();
}
#endif
