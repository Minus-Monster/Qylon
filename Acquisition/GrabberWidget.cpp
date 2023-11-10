#ifdef GRABBER_ENABLED
#include "GrabberWidget.h"
#include "Grabber.h"

Qylon::GrabberWidget::GrabberWidget(Grabber *obj): parent(obj)
{
    setWindowTitle("Basler Frame Grabber Configuration");
    setWindowIcon(QIcon(":/Qylon/Resources/Icon.png"));

    layout = new QVBoxLayout(this);
    setLayout(layout);
    setMinimumWidth(400);



    // GroupBox 'Initialization'
    {
        QVBoxLayout *layoutLine1 = new QVBoxLayout;
        QHBoxLayout *layoutApplet = new QHBoxLayout;
        QPushButton *buttonApplet = new QPushButton("Load");
        connect(buttonApplet, &QPushButton::clicked, this, [=](){
            auto get = QFileDialog::getOpenFileName(this, "Load an applet", QDir::homePath(), "*.hap *.dll");
            if(get.isEmpty()) return;

            this->lineApplet->setText(get);
            this->parent->loadApplet(get);
        });
        lineApplet = new QLineEdit;
        lineApplet->setPlaceholderText("Load an applet");
        layoutApplet->addWidget(buttonApplet);
        layoutApplet->addWidget(lineApplet);

        QHBoxLayout *layoutConfig = new QHBoxLayout;
        QPushButton *buttonConfig = new QPushButton("Load");
        connect(buttonConfig, &QPushButton::clicked, this, [=](){
            auto get = QFileDialog::getOpenFileName(this, "Load a configuration file", QDir::homePath(), "*.mcf");
            if(get.isEmpty()) return;

            this->lineConfig->setText(get);
            this->parent->loadConfiguration(get);
        });
        lineConfig = new QLineEdit;
        lineConfig->setPlaceholderText("Load a configuration file");
        layoutConfig->addWidget(buttonConfig);
        layoutConfig->addWidget(lineConfig);

        layoutLine1->addLayout(layoutApplet);
        layoutLine1->addLayout(layoutConfig);



        QVBoxLayout *layoutLine2 = new QVBoxLayout;
        QLabel *labelInit = new QLabel("DMA");
        QSpinBox *spinBoxInitCount = new QSpinBox;
        spinBoxInitCount->setRange(1, 4);
        QHBoxLayout *layoutDMA = new QHBoxLayout;
        layoutDMA->addWidget(labelInit);
        layoutDMA->addWidget(spinBoxInitCount);


        QPushButton *buttonInit = new QPushButton("Initialize");
        connect(buttonInit, &QPushButton::clicked, this, [=](){
            this->parent->initialize(spinBoxInitCount->value());
        });

        layoutLine2->addLayout(layoutDMA);
        layoutLine2->addWidget(buttonInit);
        QGroupBox *boxInit = new QGroupBox("Initialization");
        QHBoxLayout *layoutBoxInit = new QHBoxLayout;
        layout->addWidget(boxInit);
        boxInit->setLayout(layoutBoxInit);
        layoutBoxInit->addLayout(layoutLine1);
        layoutBoxInit->addLayout(layoutLine2);
    }

    // GroupBox 'Configuration'
    {
        QGroupBox *boxConfig = new QGroupBox("Configurations");
        QVBoxLayout *layoutBoxConfig = new QVBoxLayout;
        boxConfig->setLayout(layoutBoxConfig);
        layout->addWidget(boxConfig);

        // ROI GroupBox
        {
            QGroupBox *boxRoi = new QGroupBox("ROI");
            QHBoxLayout *layoutBoxRoi = new QHBoxLayout;
            boxRoi->setLayout(layoutBoxRoi);
            layoutBoxConfig->addWidget(boxRoi);

            QHBoxLayout *layoutRoiX = new QHBoxLayout;
            QLabel *labelRoiX = new QLabel("X");
            QSpinBox *spinBoxRoiX = new QSpinBox;
            spinBoxRoiX->setRange(1, 99999);
            layoutRoiX->addWidget(labelRoiX);
            layoutRoiX->addWidget(spinBoxRoiX);

            QHBoxLayout *layoutRoiY = new QHBoxLayout;
            QLabel *labelRoiY = new QLabel("Y");
            QSpinBox *spinBoxRoiY = new QSpinBox;
            spinBoxRoiY->setRange(1, 99999);
            layoutRoiY->addWidget(labelRoiY);
            layoutRoiY->addWidget(spinBoxRoiY);

            layoutBoxRoi->addLayout(layoutRoiX);
            layoutBoxRoi->addLayout(layoutRoiY);
        }

        // MCF Save
        {
            QLabel *labelMFCSave = new QLabel("Current MCF");
            QPushButton *buttonMCFSave = new QPushButton("Save");
        }
    }

}

void Qylon::GrabberWidget::setDefaultAppletPath(QString path)
{
    defaultAppletPath = path;
    this->lineApplet->setText(path);
    this->parent->loadApplet(path);
}

void Qylon::GrabberWidget::setDefaultConfigPath(QString path)
{
    defaultConfigPath = path;
    this->lineConfig->setText(path);
    this->parent->loadConfiguration(path);
}
#endif
