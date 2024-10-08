#include "GraphicsWidget.h"
#ifdef TIFF_ENABLED
#include "TiffReader.h"
#endif
#include "Processing/ImageTools.h"

Qylon::GraphicsWidget::GraphicsWidget(QWidget *parent)
    : settings(new GraphicsWidgetSettings(this))
    , view(new GraphicsView(this))
    , labelCoordinateX(new QLabel)
    , labelCoordinateY(new QLabel)
    , labelPixelColor(new QLabel)
    , labelImageInformation(new QLabel)
    , lineEditPixelColor(new QLineEdit)
{
    setParent(parent);
    if(parent!=nullptr) setWindowTitle(parent->windowTitle());
    setWindowIcon(QIcon(":/Resources/Icon.png"));
    setWindowTitle("Graphics Viewer");

    toolBar.setWindowTitle("Image Tools");
    toolBar.layout()->setSpacing(1);
    toolBar.setIconSize(QSize(24,24));
    toolBar.setStyleSheet("QToolBar{border: 1px solid lightgray; border-bottom: 1px solid lightgray;}");
    view->setStyleSheet("QGraphicsView{border-style:none; border-top: 1px solid lightgray; border-left:1px solid lightgray; border-right:1px solid lightgray;}");
    statusBar.setSizeGripEnabled(true);
    statusBar.setStyleSheet("QStatusBar::item { border: none; } QStatusBar{border:1px solid lightgray;}");

    setLayout(&layout);
    layout.setSpacing(0);
    layout.setContentsMargins(0,0,0,0);

    labelCoordinateX->setFixedWidth(45);
    labelCoordinateY->setFixedWidth(45);

    labelPixelColor->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    labelPixelColor->setFixedWidth(110);

    lineEditPixelColor->setFixedWidth(13);
    lineEditPixelColor->setFixedHeight(13);
    lineEditPixelColor->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);
    lineEditPixelColor->setReadOnly(true);
    lineEditPixelColor->setFrame(false);
    lineEditPixelColor->setHidden(true);

    settings->setBaseSize(500,500);

    connect(&fpsTimer, &QTimer::timeout, this, [&](){
        labelFPS.setText(QString::number(timerCnt) +" fps ");
        timerCnt=0;
    });
    initialize();
}

Qylon::GraphicsWidget::~GraphicsWidget(){
    view->deleteLater();
}

void Qylon::GraphicsWidget::initialize(){
    QWidget *spacer = new QWidget;
    spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    toolBar.addWidget(spacer);

    QAction *actionLoad = new QAction(QIcon(":/Resources/Icon/icons8-image-file-add-48.png"),"Load an image", this);
    toolBar.addAction(actionLoad);
    connect(actionLoad, &QAction::triggered, this, [=](){
        auto fileName = QFileDialog::getOpenFileName(this, "Load an image", QDir::currentPath(), "Image(*.png *.jpg *.jpeg *bmp *.tiff *.tif)");
        if(fileName.isEmpty()) return;

        if(fileName.contains(".tiff") || fileName.contains(".tif")){
#ifdef TIFF_ENABLED
            this->setImage(openTiff(fileName));
#else
                this->setImage(QImage(fileName));
#endif
        }else this->setImage(QImage(fileName));
    });

    QAction *actionSave = new QAction(QIcon(":/Resources/Icon/icons8-save-as-48.png"), "Save this image", this);
    actionSave->setShortcut(QKeySequence::Save);
    toolBar.addAction(actionSave);
    connect(actionSave, &QAction::triggered, this, [=](){
        auto filePath = QFileDialog::getSaveFileName(this, "Save this image", QDir::currentPath() + "/untitled.png", "Image(*.png *.jpg *.bmp *.tif *.tiff)");
        if(filePath.isEmpty()) return;

        if(this->currentImage.save(filePath))
            QMessageBox::information(this, this->windowTitle(), "This current image was successfully saved.");
        else
            QMessageBox::critical(this, this->windowTitle(), "Failed to save this current image.");

    });

    QAction *actionSettings = new QAction(QIcon(":/Resources/Icon/icons8-setting-48.png"), "Settings", this);
    toolBar.addAction(actionSettings);
    connect(actionSettings, &QAction::triggered, this, [=](){
        settings->show();
    });
    connect(settings, &GraphicsWidgetSettings::setBitShift, this, &GraphicsWidget::setBitShift);

    toolBar.addSeparator();

    QAction *actionLineProfile = new QAction(QIcon(":/Resources/Icon/icons8-measure-48.png"), "Line Profile", this);
    toolBar.addAction(actionLineProfile);
    actionLineProfile->setCheckable(true);
    connect(actionLineProfile, &QAction::toggled, this, [=](bool on){
        if(view->getCurrentImage().isNull()){
            actionLineProfile->setChecked(false);
            return;
        }
        view->setLineProfileMode(on);
    });

    QAction *actionHistogram = new QAction(QIcon(":/Resources/Icon/icons8-histogram-48.png"), "Histogram", this);
    toolBar.addAction(actionHistogram);
    actionHistogram->setCheckable(true);
    connect(actionHistogram, &QAction::toggled, this, [=](bool on){
        if(view->getCurrentImage().isNull()){
            actionHistogram->setChecked(false);
            return;
        }
        view->setHistogramMode(on);
    });
    connect(view, &GraphicsView::histogramWidgetClosed, this, [=]{
        actionHistogram->blockSignals(true);
        actionHistogram->setChecked(false);
        actionHistogram->blockSignals(false);
    });

    QAction *actionGridLine = new QAction(QIcon(":/Resources/Icon/icons8-crosshair-48.png"), "Crosshair", this);
    actionGridLine->setCheckable(true);
    toolBar.addAction(actionGridLine);
    connect(actionGridLine, &QAction::toggled, this, [=](bool on){
        if(view->getCurrentImage().isNull()){
            actionGridLine->setChecked(false);
            return;
        }
        setCrossHair(on, settings->getCrosshairColor(), settings->getCrosshairWidth());
    });


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

    QAction *actionZoomIn = new QAction(QIcon(":/Resources/Icon/icons8-zoom-in-48.png"), "Zoom In");
    connect(actionZoomIn, &QAction::triggered, this, [=](){
        this->view->setScale((float)1.2);
    });
    toolBar.addAction(actionZoomIn);

    QAction *actionZoomOut = new QAction(QIcon(":/Resources/Icon/icons8-zoom-out-48.png"), "Zoom Out");
    connect(actionZoomOut, &QAction::triggered, this, [=](){
        this->view->setScale((float)0.8);
    });
    toolBar.addAction(actionZoomOut);

    QAction *actionOriginal = new QAction(QIcon(":/Resources/Icon/icons8-one-to-one-48.png"), "Original Size");
    connect(actionOriginal, &QAction::triggered, this, [=](){
        this->view->resetScale();
    });
    toolBar.addAction(actionOriginal);

    QIcon fitIcon;
    fitIcon.addPixmap(QPixmap(":/Resources/Icon/icons8-expand-48.png"), QIcon::Normal, QIcon::Off);
    fitIcon.addPixmap(QPixmap(":/Resources/Icon/icons8-collapse-48.png"), QIcon::Normal, QIcon::On);
    QAction *actionFit = new QAction(QIcon(fitIcon), "Fit");
    actionFit->setCheckable(true);
    actionFit->setChecked(false);
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

    connect(view, &GraphicsView::currentPos, this, [=](QPointF point) {
        auto rPos = point.toPoint();
        int r, g, b = 0;
        view->getCurrentImage().pixelColor(rPos.x(), rPos.y()).getRgb(&r, &g, &b);
        this->labelCoordinateX->setText(" X: " + QString::number(rPos.x()));
        this->labelCoordinateY->setText("Y: " + QString::number(rPos.y()));

        // auto corr = (int)((r + g + b) / 3) > 150 ? 0 : 255;
        QString style =  QString("QLineEdit { background-color : rgb(") + QString::number(r) + ", " + QString::number(g) + ", " + QString::number(b) + QString("); }");
        this->lineEditPixelColor->setStyleSheet(style);
        if(this->lineEditPixelColor->isHidden()) this->lineEditPixelColor->setHidden(false);

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
                color = "RGB[" + QString::number(r) + "," + QString::number(g) + "," + QString::number(b) + "]";
            }
        }
        this->labelPixelColor->setText(color);
    });

    statusBar.addWidget(labelCoordinateX);
    statusBar.addWidget(labelCoordinateY);
    statusBar.addWidget(lineEditPixelColor);
    statusBar.addWidget(labelPixelColor);
    statusBar.addWidget(labelImageInformation);
    statusBar.addPermanentWidget(&labelFPS);

    // Putting Graphics View
    layout.addWidget(&toolBar);
    layout.addWidget(view);
    layout.addWidget(&statusBar);
}

void Qylon::GraphicsWidget::addWidget(QWidget *widget)
{
    auto action = toolBar.actions().front();
    toolBar.insertWidget(action, widget);
}

void Qylon::GraphicsWidget::setToolBarEnable(bool on){
    toolBar.setHidden(!on);
}

void Qylon::GraphicsWidget::clear()
{
    view->clear();
}

void Qylon::GraphicsWidget::reset()
{
    view->reset();
}

void Qylon::GraphicsWidget::setLogo(bool on){
    view->setLogo(on);
}

void Qylon::GraphicsWidget::setCrossHair(bool on, QColor color, int width)
{
    crosshair = on;
    color = settings->getCrosshairColor();
    width = settings->getCrosshairWidth();

    view->setCrossHair(crosshair, color, width);
    if(!on){
        for(int i=0; i< toolBar.actions().size(); ++i){
            if(toolBar.actions().at(i)->toolTip() == "Crosshair"){
                toolBar.actions().at(i)->setChecked(false);
            }
        }
    }
}

void Qylon::GraphicsWidget::setImage(const QImage image){
    QMutexLocker locker(&mutex);

    ++timerCnt;
    currentImage = image;
    QString colorFormat;
    switch(image.pixelFormat().colorModel()){
    case QPixelFormat::RGB: colorFormat="RGB"; break;
    case QPixelFormat::BGR: colorFormat="BGR"; break;
    case QPixelFormat::Indexed: colorFormat="Indexed"; break;
    case QPixelFormat::Grayscale: colorFormat="Grayscale"; break;
    case QPixelFormat::CMYK: colorFormat="CMYK"; break;
    case QPixelFormat::HSL: colorFormat="HSL"; break;
    case QPixelFormat::HSV: colorFormat="HSV"; break;
    case QPixelFormat::YUV: colorFormat="YUV"; break;
    case QPixelFormat::Alpha: colorFormat="Alpha"; break;
    default: colorFormat="Unknowned"; break;
    }

    if(bitShift !=0 && image.format() == QImage::Format_Grayscale16){
        view->setImage(shiftImage(image, bitShift));
    }else{
        view->setImage(image);
    }
    labelImageInformation->setText(QString("%1-bit %2 (%3C), %4x%5")
                                       .arg(image.depth()).arg(colorFormat)
                                       .arg(image.pixelFormat().channelCount()).arg(image.width()).arg(image.height()));


    view->setFit(view->isFit());
    view->setLogo(false);
}

void Qylon::GraphicsWidget::updateImage()
{
    setImage(currentImage);
}

bool Qylon::GraphicsWidget::drawGraphicsItem(QGraphicsItem *item)
{
    return view->addGraphicsItem(item);
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
