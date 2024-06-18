#ifdef GRABBER_ENABLED
#include "GrabberWidget.h"
#include "Grabber.h"
#include <QTabWidget>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QHeaderView>
#include <QAction>
#include <QMessageBox>

Qylon::GrabberWidget::GrabberWidget(Grabber *obj): parent(obj)
{
    setWindowTitle("Basler Frame Grabber Configuration");
    setWindowIcon(QIcon(":/Qylon/Resources/Icon.png"));

    layout = new QVBoxLayout(this);
    setLayout(layout);
    setMinimumWidth(200);

    // GroupBox 'Initialization'

    QVBoxLayout *layoutLoadFiles = new QVBoxLayout;
    QHBoxLayout *layoutLoadApplet = new QHBoxLayout;
    QPushButton *buttonLoadApplet = new QPushButton;
    buttonLoadApplet->setFlat(true);
    buttonLoadApplet->setIcon(QIcon(":/Resources/Icon/icons8-opened-folder-48.png"));
    connect(buttonLoadApplet, &QPushButton::clicked, this, [=](){
        auto get = QFileDialog::getOpenFileName(this, "Load an applet", QDir::homePath(), "*.hap *.dll");
        if(get.isEmpty()) return;

        this->lineLoadApplet->setText(get);
        this->parent->loadApplet(get);
    });
    lineLoadApplet = new QLineEdit;
    lineLoadApplet->setPlaceholderText("Load an applet");
    layoutLoadApplet->addWidget(buttonLoadApplet);
    layoutLoadApplet->addWidget(lineLoadApplet);

    QHBoxLayout *layoutLoadConfig = new QHBoxLayout;
    QPushButton *buttonLoadConfig = new QPushButton;
    buttonLoadConfig->setFlat(true);
    buttonLoadConfig->setIcon(QIcon(":/Resources/Icon/icons8-opened-folder-48.png"));

    connect(buttonLoadConfig, &QPushButton::clicked, this, [=](){
        auto get = QFileDialog::getOpenFileName(this, "Load a configuration file", QDir::homePath(), "*.mcf");
        if(get.isEmpty()) return;

        this->lineLoadConfig->setText(get);
        this->parent->loadConfiguration(get);
        this->getMCFStructure(get);

    });
    lineLoadConfig = new QLineEdit;
    lineLoadConfig->setPlaceholderText("Load a configuration file");
    layoutLoadConfig->addWidget(buttonLoadConfig);
    layoutLoadConfig->addWidget(lineLoadConfig);

    layoutLoadFiles->addLayout(layoutLoadApplet);
    layoutLoadFiles->addLayout(layoutLoadConfig);

    QPushButton *buttonInit = new QPushButton("Initialize");
    layoutLoadFiles->addWidget(buttonInit);

    QGroupBox *groupBoxInit = new QGroupBox("Initialization");
    QHBoxLayout *layoutInit = new QHBoxLayout;
    layout->addWidget(groupBoxInit);
    groupBoxInit->setLayout(layoutInit);
    layoutInit->addLayout(layoutLoadFiles);


    // GroupBox 'Configuration'

    QGroupBox *groupBoxConfigurations = new QGroupBox("Configurations");
    QVBoxLayout *layoutConfigurations = new QVBoxLayout;
    groupBoxConfigurations->setLayout(layoutConfigurations);
    groupBoxConfigurations->setVisible(false);
    layout->addWidget(groupBoxConfigurations);


    QTabWidget *tabWidget = new QTabWidget;
    layoutConfigurations->addWidget(tabWidget);

    QPushButton *pushButtonEditMCF = new QPushButton("MCF Editor");
    connect(pushButtonEditMCF, &QPushButton::clicked, this, [&](){
        mcfDialog->exec();
    });
    // connect(mcfWidget, &QTreeWidget::, this, [&](){

    // });
    layoutConfigurations->addWidget(pushButtonEditMCF);

    connect(buttonInit, &QPushButton::clicked, this, [=](){
        tabWidget->clear();
        if(!this->parent->initialize(2)) return;

        groupBoxConfigurations->setVisible(true);

        int cntDMA =this->parent->getDMACount();
        for(int i=0; i<cntDMA; ++i){
            QGroupBox *groupBox = new QGroupBox;

            // ROI
            QHBoxLayout *layoutWidth = new QHBoxLayout;
            QLabel *labelWidth = new QLabel("Width:");
            QSpinBox *spinBoxWidth = new QSpinBox;
            spinBoxWidth->setRange(0, 99999999);
            spinBoxWidth->setValue(this->parent->getWidth(i));
            connect(spinBoxWidth, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int val){
                spinBoxWidth->blockSignals(true);
                this->parent->setParameterValue(FG_WIDTH, val, i);
                spinBoxWidth->setValue(this->parent->getWidth(i));
                spinBoxWidth->blockSignals(false);
            });
            layoutWidth->addWidget(labelWidth);
            layoutWidth->addWidget(spinBoxWidth);

            QHBoxLayout *layoutHeight = new QHBoxLayout;
            QLabel *labelHeight = new QLabel("Height");
            QSpinBox *spinBoxHeight = new QSpinBox;
            spinBoxHeight->setRange(0, 99999999);
            spinBoxHeight->setValue(this->parent->getHeight(i));
            connect(spinBoxHeight, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int val){
                spinBoxHeight->blockSignals(true);
                this->parent->setParameterValue(FG_HEIGHT, val, i);
                spinBoxHeight->setValue(this->parent->getHeight(i));
                spinBoxHeight->blockSignals(false);
            });
            layoutHeight->addWidget(labelHeight);
            layoutHeight->addWidget(spinBoxHeight);

            QVBoxLayout *layoutSize = new QVBoxLayout;
            layoutSize->addLayout(layoutWidth);
            layoutSize->addLayout(layoutHeight);

            QHBoxLayout *layoutX = new QHBoxLayout;
            QLabel *labelX= new QLabel("X:");
            QSpinBox *spinBoxX = new QSpinBox;
            spinBoxX->setRange(0, 99999999);
            spinBoxX->setValue(this->parent->getX(i));
            connect(spinBoxX, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int val){
                spinBoxX->blockSignals(true);
                this->parent->setParameterValue(FG_XOFFSET, val, i);
                spinBoxX->setValue(this->parent->getX(i));
                spinBoxX->blockSignals(false);
            });
            layoutX->addWidget(labelX);
            layoutX->addWidget(spinBoxX);

            QHBoxLayout *layoutY = new QHBoxLayout;
            QLabel *labelY= new QLabel("Y:");
            QSpinBox *spinBoxY = new QSpinBox;
            spinBoxY->setRange(0, 99999999);
            spinBoxY->setValue(this->parent->getY(i));
            connect(spinBoxY, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int val){
                spinBoxY->blockSignals(true);
                this->parent->setParameterValue(FG_YOFFSET, val, i);
                spinBoxY->setValue(this->parent->getY(i));
                spinBoxY->blockSignals(false);
            });
            layoutY->addWidget(labelY);
            layoutY->addWidget(spinBoxY);

            QVBoxLayout *layoutOffset = new QVBoxLayout;
            layoutOffset->addLayout(layoutX);
            layoutOffset->addLayout(layoutY);

            QHBoxLayout *layoutROI = new QHBoxLayout;
            layoutROI->addLayout(layoutSize);
            layoutROI->addLayout(layoutOffset);


            QVBoxLayout *layoutGroupBox = new QVBoxLayout;
            layoutGroupBox->addLayout(layoutROI);

            groupBox->setLayout(layoutGroupBox);
            groupBox->setFlat(true);
            tabWidget->addTab(groupBox, "DMA:" + QString::number(i));
        }
    });


}

void Qylon::GrabberWidget::setDefaultAppletPath(QString path)
{
    defaultAppletPath = path;
    this->lineLoadApplet->setText(path);
    this->parent->loadApplet(path);
}

void Qylon::GrabberWidget::setDefaultConfigPath(QString path)
{
    defaultConfigPath = path;
    this->lineLoadConfig->setText(path);
    this->parent->loadConfiguration(path);
}

void Qylon::GrabberWidget::setDMACount(int val)
{
    emit changedDMACount(val);
}

void Qylon::GrabberWidget::getMCFStructure(QString mcfPath)
{
    QFile file(mcfPath);
    qDebug() << "Start analyzing " << mcfPath;

    QString sectionParser = "[";
    QString valueParser = "=";

    if(mcfDialog != nullptr) mcfDialog->deleteLater();
    mcfDialog = new QDialog(this);
    QVBoxLayout *mcfLayout = new QVBoxLayout;
    mcfDialog->setLayout(mcfLayout);

    QLineEdit *lineEditSearch = new QLineEdit;
    lineEditSearch->setFrame(true);
    lineEditSearch->setPlaceholderText("Search the parameter");
    lineEditSearch->setClearButtonEnabled(true);
    lineEditSearch->setStyleSheet("QLineEdit{ border: 1px solid gray; height: 20px; }");
    mcfLayout->addWidget(lineEditSearch);


    QTreeWidget *widget = new QTreeWidget(mcfDialog);
    widget->setHeaderLabels(QStringList() << "Parameter" << "Value");
    widget->header()->resizeSection(0, 200);
    mcfLayout->addWidget(widget);

    mcfDialog->setMinimumSize(400,300);
    widget->setMinimumSize(400,300);

    if(file.open(QIODevice::ReadOnly | QIODevice::Text)){
        QTextStream in(&file);
        QTreeWidgetItem *currentParent=nullptr;
        while(!in.atEnd()){
            QString line = in.readLine();
            if(line.isEmpty()) continue;

            if(line.contains(sectionParser)){
                QTreeWidgetItem *section = new QTreeWidgetItem(widget);
                section->setText(0, line.remove("[").remove("]"));
                currentParent = section;
                continue;
            }
            auto values = line.split(valueParser);

            QTreeWidgetItem *children = new QTreeWidgetItem(currentParent);
            children->setText(0, values.first());

            QLineEdit *lineEditParameterValue = new QLineEdit(values.last().remove(";"));
            lineEditParameterValue->setFrame(false);
            lineEditParameterValue->setObjectName(values.first());
            connect(lineEditParameterValue, &QLineEdit::returnPressed, [lineEditParameterValue, currentParent, this](){
                int r = QMessageBox::question(nullptr, "MCF Editor", "Are you sure to change this value?", QMessageBox::Cancel | QMessageBox::Ok, QMessageBox::Cancel);
                if(r == QMessageBox::Cancel){
                    lineEditParameterValue->undo();
                    return;
                }
                QString parent = currentParent->text(0);
                QString obj = lineEditParameterValue->objectName();
                QString filterString = "ID";
                if(parent.startsWith(filterString)){
                    parent = parent.remove(filterString);
                    bool isNumeric = false;
                    lineEditParameterValue->text().toInt(&isNumeric);
                    if(isNumeric){
                        this->parent->setParameterValue(obj, lineEditParameterValue->text().toInt(), parent.toInt());
                    }

                    qDebug() << "Name" << obj << "DMA" << parent.toInt() << "value" << lineEditParameterValue->text().toInt();
                }
            });

            widget->setItemWidget(children, 1, lineEditParameterValue);
        }
    }

    auto filterItems = [widget](const QString &searchText) {
        for (int i = 0; i < widget->topLevelItemCount(); ++i) {
            QTreeWidgetItem *topItem = widget->topLevelItem(i);
            bool matchFound = false;
            for (int j = 0; j < topItem->childCount(); ++j) {
                QTreeWidgetItem *childItem = topItem->child(j);
                if (childItem->text(0).contains(searchText, Qt::CaseInsensitive) ||
                    qobject_cast<QLineEdit*>(widget->itemWidget(childItem, 1))->text().contains(searchText, Qt::CaseInsensitive)) {
                    matchFound = true;
                    childItem->setHidden(false);
                } else {
                    childItem->setHidden(true);
                }
            }
            topItem->setHidden(!matchFound);
            topItem->setExpanded(matchFound);
        }
    };
    connect(lineEditSearch, &QLineEdit::textChanged, filterItems);


}
#endif
