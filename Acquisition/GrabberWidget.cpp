#ifdef GRABBER_ENABLED
#include "GrabberWidget.h"
#include "Grabber.h"
#include "Qylon.h"
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
    setMinimumWidth(250);

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
    });
    lineLoadConfig = new QLineEdit;
    lineLoadConfig->setPlaceholderText("Load a configuration file");
    layoutLoadConfig->addWidget(buttonLoadConfig);
    layoutLoadConfig->addWidget(lineLoadConfig);

    layoutLoadFiles->addLayout(layoutLoadApplet);
    layoutLoadFiles->addLayout(layoutLoadConfig);

    QSpinBox *spinBoxImageBuffer = new QSpinBox;
    spinBoxImageBuffer->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    spinBoxImageBuffer->setRange(1,10);
    spinBoxImageBuffer->setValue(3);
    QHBoxLayout *layoutImageBuffer = new QHBoxLayout;
    layoutImageBuffer->addWidget(new QLabel("Image Buffer"));
    layoutImageBuffer->addWidget(spinBoxImageBuffer);
    layoutLoadFiles->addLayout(layoutImageBuffer);

    QPushButton *buttonInit = new QPushButton("Initialization");
    buttonInit->setCheckable(true);
    layoutLoadFiles->addWidget(buttonInit);

    QGroupBox *groupBoxInit = new QGroupBox("Initialization");
    QHBoxLayout *layoutInit = new QHBoxLayout;
    layout->addWidget(groupBoxInit);
    groupBoxInit->setLayout(layoutInit);
    layoutInit->addLayout(layoutLoadFiles);
    groupBoxInit->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);

    // GroupBox 'Configuration'
    QGroupBox *groupBoxConfigurations = new QGroupBox("Configurations");
    layout->addWidget(groupBoxConfigurations);
    QVBoxLayout *layoutConfigurations = new QVBoxLayout;
    groupBoxConfigurations->setLayout(layoutConfigurations);
    groupBoxConfigurations->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    tabWidgetDMA = new QTabWidget;
    layoutConfigurations->addWidget(tabWidgetDMA);
    QPushButton *pushButtonEditMCF = new QPushButton("MCF Editor");
    pushButtonEditMCF->setEnabled(false);
    connect(pushButtonEditMCF, &QPushButton::clicked, this, [&](){
        mcfEditor->exec();
    });
    layoutConfigurations->addWidget(pushButtonEditMCF);
    connect(buttonInit, &QPushButton::clicked, this, [=](bool checked){
        if(!checked){
            parent->release();
            tabWidgetDMA->clear();
        }else{
            if(!parent->isInitialized()){
                parent->loadApplet(this->lineLoadApplet->text());
                parent->loadConfiguration(this->lineLoadConfig->text());
                parent->initialize(spinBoxImageBuffer->value());
            }
            initTabWidget();
        }
    });    
    layout->addSpacerItem(new QSpacerItem(0, 10, QSizePolicy::Minimum, QSizePolicy::Expanding));

    connect(obj, &Grabber::loadedApplet, lineLoadApplet, &QLineEdit::setText);
    connect(obj, &Grabber::loadedConfig, lineLoadConfig, &QLineEdit::setText);
    connect(obj, &Grabber::initializingState, this, [=](bool on){
        buttonInit->setChecked(on);
        spinBoxImageBuffer->setEnabled(!on);
        lineLoadApplet->setEnabled(!on);
        lineLoadConfig->setEnabled(!on);
        buttonLoadApplet->setEnabled(!on);
        buttonLoadConfig->setEnabled(!on);
        pushButtonEditMCF->setEnabled(on);
        if(on){
            initTabWidget();
            getMCFStructure(lineLoadConfig->text());
        }else{
            tabWidgetDMA->clear();
        }
    });
    connect(obj, &Grabber::grabbingState, this, [=](bool on){
        pushButtonEditMCF->setEnabled(!on);
        tabWidgetDMA->setEnabled(!on);
    });
}

void Qylon::GrabberWidget::initTabWidget()
{
    tabWidgetDMA->clear();
    if(!parent->isInitialized()) return;

    int cntDMA =this->parent->getDMACount();
    for(int i=0; i<cntDMA; ++i){
        QGroupBox *groupBox = new QGroupBox;
        groupBox->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);

        // ROI
        QHBoxLayout *layoutWidth = new QHBoxLayout;
        QLabel *labelWidth = new QLabel("Width:");
        QSpinBox *spinBoxWidth = new QSpinBox;
        spinBoxWidth->setRange(0, 99999999);
        spinBoxWidth->setValue(this->parent->getWidth(i));
        spinBoxWidth->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        connect(spinBoxWidth, &QSpinBox::editingFinished, this, [=]{
            spinBoxWidth->blockSignals(true);
            this->parent->setParameterValue(FG_WIDTH, spinBoxWidth->value(), i);
            spinBoxWidth->setValue(this->parent->getWidth(i));
            this->refreshMCFValues();
            spinBoxWidth->blockSignals(false);
        });
        layoutWidth->addWidget(labelWidth);
        layoutWidth->addWidget(spinBoxWidth);

        QHBoxLayout *layoutHeight = new QHBoxLayout;
        QLabel *labelHeight = new QLabel("Height");
        QSpinBox *spinBoxHeight = new QSpinBox;
        spinBoxHeight->setRange(0, 99999999);
        spinBoxHeight->setValue(this->parent->getHeight(i));
        spinBoxHeight->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        connect(spinBoxHeight, &QSpinBox::editingFinished, this, [=]{
            spinBoxHeight->blockSignals(true);
            this->parent->setParameterValue(FG_HEIGHT, spinBoxHeight->value(), i);
            spinBoxHeight->setValue(this->parent->getHeight(i));
            this->refreshMCFValues();
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
        spinBoxX->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
        connect(spinBoxX, &QSpinBox::editingFinished, this, [=]{
            spinBoxX->blockSignals(true);
            this->parent->setParameterValue(FG_XOFFSET, spinBoxX->value(), i);
            spinBoxX->setValue(this->parent->getX(i));
            this->refreshMCFValues();
            spinBoxX->blockSignals(false);
        });
        layoutX->addWidget(labelX);
        layoutX->addWidget(spinBoxX);

        QHBoxLayout *layoutY = new QHBoxLayout;
        QLabel *labelY= new QLabel("Y:");
        QSpinBox *spinBoxY = new QSpinBox;
        spinBoxY->setRange(0, 99999999);
        spinBoxY->setValue(this->parent->getY(i));
        spinBoxY->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
        connect(spinBoxY, &QSpinBox::editingFinished, this, [=]{
            spinBoxY->blockSignals(true);
            this->parent->setParameterValue(FG_YOFFSET, spinBoxY->value(), i);
            spinBoxY->setValue(this->parent->getY(i));
            this->refreshMCFValues();
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
        tabWidgetDMA->addTab(groupBox, "DMA:" + QString::number(i));
    }
    tabWidgetDMA->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
}

void Qylon::GrabberWidget::getMCFStructure(QString mcfPath)
{
    Qylon::log("Generating MCF Editor");
    QFile file(mcfPath);
    QString sectionParser = "[";
    QString valueParser = "=";

    if(mcfEditor != nullptr) mcfEditor->deleteLater();
    mcfEditor = new QDialog(this);
    QVBoxLayout *mcfLayout = new QVBoxLayout;
    mcfEditor->setLayout(mcfLayout);

    QLineEdit *lineEditSearch = new QLineEdit;
    lineEditSearch->setFrame(true);
    lineEditSearch->setPlaceholderText("Search the parameter");
    lineEditSearch->setClearButtonEnabled(true);
    lineEditSearch->setStyleSheet("QLineEdit{ border: 1px solid gray; height: 20px; }");
    mcfLayout->addWidget(lineEditSearch);


    QTreeWidget *widget = new QTreeWidget(mcfEditor);
    widget->setHeaderLabels(QStringList() << "Parameter" << "Value");
    widget->header()->resizeSection(0, 200);
    mcfLayout->addWidget(widget);
    QPushButton *saveButton = new QPushButton("Save MCF");
    mcfLayout->addWidget(saveButton);
    saveButton->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    connect(saveButton, &QPushButton::clicked, this, [this]() {
        QString savePath = QFileDialog::getSaveFileName(this, "Save MCF", QDir::currentPath(), "MCF Files (*.mcf);;All Files (*)");
        if (!savePath.isEmpty()) {
            if(parent->saveCurrentConfig(savePath)){
                QMessageBox::information(this, "MCF Editor", "File saved successfully.");
            }else{
                QMessageBox::warning(this, "MCF Editor", "Failed to save the mcf file.");
            }
        }
    });

    mcfEditor->setMinimumSize(400,300);
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
            connect(lineEditParameterValue, &QLineEdit::returnPressed, this, [lineEditParameterValue, currentParent, this](){
                int r = QMessageBox::question(nullptr, "MCF Editor", "Are you sure to change this value?", QMessageBox::Cancel | QMessageBox::Ok, QMessageBox::Cancel);
                if(r == QMessageBox::Cancel){
                    lineEditParameterValue->undo();
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
                        Qylon::log(obj + QString(" is changed to " + QString::number(this->parent->getParameterIntValue(obj, parent.toInt()))));
                    }else{
                        this->parent->setParameterValue(obj, lineEditParameterValue->text(), parent.toInt());
                        Qylon::log(obj + QString(" is changed to " + this->parent->getParameterStringValue(obj, parent.toInt())));
                    }
                    this->refreshMCFValues();
                    if(obj.contains("FG_")) initTabWidget();
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

void Qylon::GrabberWidget::saveMCFStructure(QString savePath)
{
    if (!mcfEditor) return;

    QFile file(savePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "MCF Editor", "Failed to save the file.");
        return;
    }

    QTextStream out(&file);

    QTreeWidget *widget = mcfEditor->findChild<QTreeWidget*>();
    if (!widget) return;

    for (int i = 0; i < widget->topLevelItemCount(); ++i) {
        QTreeWidgetItem *section = widget->topLevelItem(i);
        out << "[" << section->text(0) << "]\n";

        for (int j = 0; j < section->childCount(); ++j) {
            QTreeWidgetItem *child = section->child(j);
            QLineEdit *lineEditParameterValue = qobject_cast<QLineEdit*>(widget->itemWidget(child, 1));
            if (lineEditParameterValue) {
                out << child->text(0) << "=" << lineEditParameterValue->text() << ";\n";
            }
        }
        if(i != widget->topLevelItemCount()-1) out << "\n";
    }

    file.close();
    QMessageBox::information(this, "MCF Editor", "File saved successfully.");
}

void Qylon::GrabberWidget::refreshMCFValues()
{
    if (!mcfEditor) return;

    QTreeWidget *widget = mcfEditor->findChild<QTreeWidget*>();
    if (!widget) return;

    for (int i = 0; i < widget->topLevelItemCount(); ++i) {
        QTreeWidgetItem *topItem = widget->topLevelItem(i);
        for (int j = 0; j < topItem->childCount(); ++j) {
            QTreeWidgetItem *childItem = topItem->child(j);
            QLineEdit *lineEdit = qobject_cast<QLineEdit*>(widget->itemWidget(childItem, 1));
            if (lineEdit) {
                QString parameter = childItem->text(0);
                QString section = topItem->text(0);
                QString filterString = "ID";
                if (section.startsWith(filterString)) {
                    section = section.remove(filterString);
                    bool isNumeric = false;
                    lineEdit->text().toInt(&isNumeric);
                    if (isNumeric) {
                        lineEdit->setText(QString::number(this->parent->getParameterIntValue(parameter, section.toInt())));
                    } else {
                        lineEdit->setText(this->parent->getParameterStringValue(parameter, section.toInt()));
                    }
                }
            }
        }
    }
}


#endif
