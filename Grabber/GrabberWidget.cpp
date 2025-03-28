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
#include <QToolButton>

Qylon::GrabberWidget::GrabberWidget(Grabber *obj): parent(obj)
{
    setWindowTitle("Basler Frame Grabber Configuration");
    setWindowIcon(QIcon(":/Resources/Icon.png"));

    layout = new QVBoxLayout(this);
    setLayout(layout);
    setMinimumWidth(270);

    QHBoxLayout *layoutLoadApplet = new QHBoxLayout;
    QToolButton *buttonLoadApplet = new QToolButton;
    buttonLoadApplet->setAutoRaise(true);
    buttonLoadApplet->setIconSize(QSize(20,20));
    buttonLoadApplet->setDefaultAction(new QAction(QIcon(":/Resources/Icon/icons8-opened-folder-48.png"), "Load applet"));
    connect(buttonLoadApplet, &QToolButton::triggered, this, [=](){
        auto get = QFileDialog::getOpenFileName(this, "Load an applet", QDir::homePath(), "*.hap *.dll *.so");
        if(get.isEmpty()) return;

        this->lineLoadApplet->setText(get);
        this->parent->loadApplet(get);
    });
    lineLoadApplet = new QLineEdit;
    lineLoadApplet->setPlaceholderText("Load an applet");
    // lineLoadApplet->setText("/opt/Basler/FramegrabberSDK/dll/mE5-MA-VCL/libAcq_SingleFullAreaGray.so");
    layoutLoadApplet->addWidget(buttonLoadApplet);
    layoutLoadApplet->addWidget(lineLoadApplet);

    QHBoxLayout *layoutLoadConfig = new QHBoxLayout;
    QToolButton *buttonEditMCF = new QToolButton;
    buttonEditMCF->setEnabled(false);
    buttonEditMCF->setAutoRaise(true);
    buttonEditMCF->setDefaultAction(new QAction(QIcon(":/Resources/Icon/icons8-note-48.png"),"MCF Editor"));
    buttonEditMCF->setIconSize(QSize(20,20));
    buttonEditMCF->setEnabled(false);
    connect(buttonEditMCF, &QToolButton::triggered, this, [&](){
        if(mcfEditor!=nullptr) mcfEditor->exec();
    });
    QToolButton *buttonLoadConfig = new QToolButton;
    buttonLoadConfig->setAutoRaise(true);
    buttonLoadConfig->setDefaultAction(new QAction(QIcon(":/Resources/Icon/icons8-opened-folder-48.png"), "Load configuration file"));
    buttonLoadConfig->setIconSize(QSize(20,20));
    connect(buttonLoadConfig, &QToolButton::triggered, this, [=](){
        auto get = QFileDialog::getOpenFileName(this, "Load a configuration file", QDir::homePath(), "*.mcf");
        if(get.isEmpty()) return;

        this->lineLoadConfig->setText(get);
        if(this->parent->loadConfiguration(get)){
            getMCFStructure(get);
        }else{
            qDebug() << "Failed to load";
        }
    });
    lineLoadConfig = new QLineEdit;
    lineLoadConfig->setPlaceholderText("Load a configuration file");

    layoutLoadConfig->addWidget(buttonLoadConfig);
    layoutLoadConfig->addWidget(lineLoadConfig);
    layoutLoadConfig->addWidget(buttonEditMCF);
    layout->addLayout(layoutLoadApplet);
    layout->addLayout(layoutLoadConfig);

    QSpinBox *spinBoxImageBuffer = new QSpinBox;
    spinBoxImageBuffer->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    spinBoxImageBuffer->setRange(1,10);
    spinBoxImageBuffer->setValue(1);
    QHBoxLayout *layoutImageBuffer = new QHBoxLayout;
    layoutImageBuffer->addWidget(new QLabel("Image Buffer"));
    layoutImageBuffer->addWidget(spinBoxImageBuffer);
    layout->addLayout(layoutImageBuffer);

    QToolButton *buttonInit = new QToolButton;
    buttonInit->setDefaultAction(new QAction(QIcon(":/Resources/Icon/icons8-software-installer-48.png"), "Set-up"));
    buttonInit->defaultAction()->setCheckable(true);
    buttonInit->setAutoRaise(true);
    buttonInit->setIconSize(QSize(20,20));
    buttonInit->setToolButtonStyle(Qt::ToolButtonStyle::ToolButtonTextBesideIcon);
    layoutImageBuffer->addStretch(100);
    layoutImageBuffer->addWidget(buttonInit);

    connect(buttonInit, &QToolButton::triggered, this, [=](QAction *action){
        if(!action->isChecked()){
            parent->release();
            layout->removeWidget(tabWidgetDMA);
            delete tabWidgetDMA;
            tabWidgetDMA = nullptr;
            layout->invalidate();
            adjustSize();
        }else{
            if(this->lineLoadApplet->text().isEmpty()){
                QMessageBox::warning(this, this->windowTitle(), "Not set an applet path. \nSet the applet file path first.");
                action->setChecked(false);
                return;
            }
            if(!parent->isInitialized()){
                if(!parent->loadApplet(this->lineLoadApplet->text())){
                    QMessageBox::warning(this, this->windowTitle(), "Applet loading failed. \nCheck the applet file or the environment.");
                    action->setChecked(false);
                    return;
                }
                parent->loadConfiguration(this->lineLoadConfig->text());
                parent->initialize(spinBoxImageBuffer->value());
            }
            initTabWidget();

        }
    });
    layout->addSpacerItem(new QSpacerItem(0, 20, QSizePolicy::Minimum, QSizePolicy::Minimum));

    connect(obj, &Grabber::loadedApplet, lineLoadApplet, &QLineEdit::setText);
    connect(obj, &Grabber::loadedConfig, lineLoadConfig, &QLineEdit::setText);
    connect(obj, &Grabber::initializingState, this, [=](bool on){
        buttonInit->setChecked(on);
        spinBoxImageBuffer->setEnabled(!on);
        lineLoadApplet->setEnabled(!on);
        lineLoadConfig->setEnabled(!on);
        buttonLoadApplet->setEnabled(!on);
        buttonLoadConfig->setEnabled(!on);
        buttonEditMCF->setEnabled(on);
    });
    connect(obj, &Grabber::grabbingState, this, [=](bool on){
        buttonEditMCF->setEnabled(!on);
        if(tabWidgetDMA != nullptr) tabWidgetDMA->setEnabled(!on);
    });
    connect(obj, &Grabber::updatedParametersValue, this, [=]{
        refreshMCFValues();
    });
}

void Qylon::GrabberWidget::initTabWidget()
{
    if(tabWidgetDMA == nullptr){
        tabWidgetDMA = new QTabWidget;
        layout->addWidget(tabWidgetDMA);
    }
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

        connect(parent, &Grabber::updatedParametersValue, this, [=]{
            int width = parent->getWidth(i);
            int height = parent->getHeight(i);
            int x = parent->getX(i);
            int y = parent->getY(i);

            spinBoxWidth->setValue(width);
            spinBoxHeight->setValue(height);
            spinBoxX->setValue(x);
            spinBoxY->setValue(y);
        });
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
    mcfEditor->setWindowTitle("MCF Editor");
    mcfEditor->setWindowIcon(this->windowIcon());

    QLineEdit *lineEditSearch = new QLineEdit;
    lineEditSearch->setFrame(true);
    lineEditSearch->setPlaceholderText("Search:");
    lineEditSearch->setClearButtonEnabled(true);
    lineEditSearch->setStyleSheet("QLineEdit{ border: 1px solid gray; height: 20px; }");


    QTreeWidget *widget = new QTreeWidget(mcfEditor);
    widget->setHeaderLabels(QStringList() << "Parameter" << "Value");
    widget->header()->resizeSection(0, 200);

    QHBoxLayout *layoutButtons = new QHBoxLayout;

    QToolButton *saveButton = new QToolButton;
    saveButton->setAutoRaise(true);
    saveButton->setDefaultAction(new QAction(QIcon(":/Resources/Icon/icons8-save-as-48.png"), "Save"));
    saveButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    saveButton->setIconSize(QSize(20,20));
    saveButton->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    connect(saveButton, &QToolButton::triggered, this, [=]{
        auto val = QMessageBox::warning(this, "MCF Editor", "Are you sure to overwrite mcf file whith this settings?",
                                        QMessageBox::Yes | QMessageBox::No);
        if(val == QMessageBox::Yes){
            parent->saveCurrentConfig(this->lineLoadConfig->text());
        }else return;
    });

    QToolButton *saveAsButton = new QToolButton;
    saveAsButton->setAutoRaise(true);
    saveAsButton->setDefaultAction(new QAction(QIcon(":/Resources/Icon/icons8-save-as-48.png"), "Save as"));
    saveAsButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    saveAsButton->setIconSize(QSize(20,20));
    saveAsButton->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    connect(saveAsButton, &QToolButton::triggered, this, [=]{
        QString savePath = QFileDialog::getSaveFileName(this, "Save MCF", QDir::currentPath(), "MCF File (*.mcf)");
        if (!savePath.isEmpty()) {
            if(parent->saveCurrentConfig(savePath)){
                QMessageBox::information(this, "MCF Editor", "File saved successfully.");
            }else{
                QMessageBox::warning(this, "MCF Editor", "Failed to save the mcf file.");
            }
        }
    });

    QToolButton *buttonRefresh = new QToolButton;
    buttonRefresh->setAutoRaise(true);
    buttonRefresh->setDefaultAction(new QAction(QIcon(":/Resources/Icon/icons8-refresh-48.png"), "Refresh"));
    buttonRefresh->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    buttonRefresh->setIconSize(QSize(20,20));
    connect(buttonRefresh, &QToolButton::triggered, this, [=]{
        refreshMCFValues();
    });

    layoutButtons->addWidget(saveButton);
    layoutButtons->addWidget(saveAsButton);
    layoutButtons->addWidget(buttonRefresh);
    layoutButtons->addStretch(100);

    mcfLayout->addLayout(layoutButtons);
    mcfLayout->addWidget(lineEditSearch);
    mcfLayout->addWidget(widget);

    mcfEditor->setMinimumSize(400,300);
    widget->setMinimumSize(400,300);

    mcfEditor->setWindowFlags(mcfEditor->windowFlags() & ~Qt::WindowContextHelpButtonHint);


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
                int r = QMessageBox::question(mcfEditor, "MCF Editor", "Are you sure to change this value?", QMessageBox::Cancel | QMessageBox::Ok, QMessageBox::Cancel);
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
                    }else{
                        this->parent->setParameterValue(obj, lineEditParameterValue->text(), parent.toInt());
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
