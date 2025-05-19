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
#include <QMenu>
#include <QStyledItemDelegate>

Qylon::GrabberWidget::GrabberWidget(Grabber *obj): parent(obj)
{
    setWindowTitle("Basler Frame Grabber Configuration");
    setWindowIcon(QIcon(":/Resources/Icon.png"));

    layout = new QVBoxLayout(this);
    layout->setContentsMargins(0,0,0,0);
    setLayout(layout);
    setMinimumWidth(270);

    QHBoxLayout *layoutLoadApplet = new QHBoxLayout;
    QToolButton *buttonLoadApplet = new QToolButton;
    buttonLoadApplet->setAutoRaise(true);
    buttonLoadApplet->setIconSize(QSize(24,24));
    buttonLoadApplet->setDefaultAction(new QAction(QIcon(":/Resources/Icon/icons8-network-card-48-5.png"), "Load applet"));
    connect(buttonLoadApplet, &QToolButton::triggered, this, [=](){
        auto get = QFileDialog::getOpenFileName(this, "Load an applet", QDir::homePath(), "*.hap *.dll *.so");
        if(get.isEmpty()) return;

        this->lineLoadApplet->setText(get);
    });
    lineLoadApplet = new QLineEdit;
    lineLoadApplet->setPlaceholderText("Load an applet");
    // lineLoadApplet->setText("/opt/Basler/FramegrabberSDK/dll/mE5-MA-VCL/libAcq_SingleFullAreaGray.so");
    layoutLoadApplet->setContentsMargins(9,9,9,5);
    layoutLoadApplet->addWidget(buttonLoadApplet);
    layoutLoadApplet->addWidget(lineLoadApplet);

    QHBoxLayout *layoutLoadConfig = new QHBoxLayout;
    QToolButton *buttonEditMCF = new QToolButton;
    buttonEditMCF->setEnabled(false);
    buttonEditMCF->setAutoRaise(true);
    buttonEditMCF->setDefaultAction(new QAction(QIcon(":/Resources/Icon/icons8-note-48.png"),"MCF Editor"));
    buttonEditMCF->setIconSize(QSize(24,24));
    buttonEditMCF->setEnabled(false);
    connect(buttonEditMCF, &QToolButton::triggered, this, [&](){
        if(mcfEditor!=nullptr) mcfEditor->exec();
    });
    QToolButton *buttonLoadConfig = new QToolButton;
    buttonLoadConfig->setAutoRaise(true);
    buttonLoadConfig->setDefaultAction(new QAction(QIcon(":/Resources/Icon/icons8-network-card-48-4.png"), "Load configuration file"));
    buttonLoadConfig->setIconSize(QSize(24,24));
    connect(buttonLoadConfig, &QToolButton::triggered, this, [=](){
        auto get = QFileDialog::getOpenFileName(this, "Load a configuration file", QDir::homePath(), "*.mcf");
        if(get.isEmpty()) return;

        this->lineLoadConfig->setText(get);
    });
    lineLoadConfig = new QLineEdit;
    lineLoadConfig->setPlaceholderText("Load a configuration file");

    layoutLoadConfig->setContentsMargins(9,0,9,5);
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
    layoutImageBuffer->setContentsMargins(9,0,9,5);
    layout->addLayout(layoutImageBuffer);

    QToolButton *buttonInit = new QToolButton;
    buttonInit->setDefaultAction(new QAction(QIcon(":/Resources/Icon/icons8-software-installer-48.png"), "Set-up"));
    buttonInit->defaultAction()->setCheckable(true);
    buttonInit->setAutoRaise(true);
    buttonInit->setIconSize(QSize(20,20));
    buttonInit->setToolButtonStyle(Qt::ToolButtonStyle::ToolButtonTextBesideIcon);

    QMenu *initMenu = new QMenu(buttonInit);
    QAction *actionWithoutAPC = new QAction("Without APC Handler(Threading)", initMenu);
    actionWithoutAPC->setCheckable(true);
    initMenu->addAction(actionWithoutAPC);
    buttonInit->setMenu(initMenu);
    buttonInit->setPopupMode(QToolButton::DelayedPopup);

    layoutImageBuffer->addStretch(100);
    layoutImageBuffer->addWidget(buttonInit);

    connect(buttonInit, &QToolButton::triggered, this, [=](QAction *action){
        if(action == actionWithoutAPC) return;

        if(!action->isChecked()){ // Release function
            parent->release();
        }else{ // Init function
            if(this->lineLoadApplet->text().isEmpty()){
                QMessageBox::warning(this, this->windowTitle(), "Not set an applet path. \nSet the applet file path first.");
                action->setChecked(false);
                return;
            }
            bool initApplet = parent->loadApplet(this->lineLoadApplet->text());
            if(initApplet){
                if(!actionWithoutAPC->isChecked()) parent->registerAPCHandler(spinBoxImageBuffer->value());
            }else{
                QMessageBox::warning(this, this->windowTitle(), "Applet loading failed. \nCheck the applet file or the environment.");
                action->setChecked(false);
                return;
            }
            if(!this->lineLoadConfig->text().isEmpty()){
                bool initConf = this->parent->loadConfiguration(this->lineLoadConfig->text());
                if(!initConf){
                    int r= QMessageBox::question(this, this->windowTitle(), "Config loading failed. \nCheck the config file or the environment.\nOr ignoring the error handling policy?");
                    if(r == QMessageBox::Yes){
                        if(this->parent->loadConfiguration(this->lineLoadConfig->text(),true)){
                            QMessageBox::information(this, this->windowTitle(), "Ignored the error handling policy. \nLoaded the configuration file.");
                        }
                    }
                }
            }
        }
    });
    layout->addSpacerItem(new QSpacerItem(0, 10  , QSizePolicy::Minimum, QSizePolicy::Minimum));

    statusBar = new QStatusBar(this);
    layout->addWidget(statusBar);

    connect(obj, &Grabber::loadedApplet, this, [=](QString path){
        lineLoadApplet->setText(path);
        lineLoadApplet->setEnabled(false);
        spinBoxImageBuffer->setEnabled(false);
        initTabWidget();
        buttonInit->setChecked(true);
    });
    connect(obj, &Grabber::loadedConfig, this, [=](QString path){
        lineLoadConfig->setText(path);
        lineLoadConfig->setEnabled(false);
        getMCFStructure(this->lineLoadConfig->text());
        buttonEditMCF->setEnabled(true);
        initTabWidget();
    });
    connect(obj, &Grabber::released, this, [=]{
        lineLoadApplet->setEnabled(true);
        lineLoadConfig->setEnabled(true);
        spinBoxImageBuffer->setEnabled(true);
        buttonEditMCF->setEnabled(false);
        buttonInit->setChecked(false);

        layout->removeWidget(tabWidgetDMA);
        delete tabWidgetDMA;
        tabWidgetDMA = nullptr;

        delete mcfEditor;
        mcfEditor = nullptr;

        layout->invalidate();
        adjustSize();
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
    // Clear existing tabs
    for(int i =0; i<tabWidgetDMA->count(); ++i){
        auto widget = tabWidgetDMA->widget(i);
        tabWidgetDMA->removeTab(i);
        widget->deleteLater();
    }

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

        connect(parent, &Grabber::updatedParametersValue, groupBox, [=]{
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
    Qylon::log("Generating MCF Editor.");
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

    mcfEditor->setWindowFlags(Qt::Window);

    if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream in(&file);
        QTreeWidgetItem* sectionItem = nullptr;

        while (!in.atEnd()) {
            QString line = in.readLine().trimmed();
            if (line.isEmpty()) continue;

            if (line.startsWith("[") && line.endsWith("]")) {
                sectionItem = new QTreeWidgetItem(widget);
                sectionItem->setText(0, line.mid(1, line.length() - 2));
                continue;
            }

            auto values = line.split(valueParser); // e.g. '='
            if (values.size() < 2) continue;

            QString fullKey = values.first().trimmed();  // e.g., Device1_Process0_AppletProperties_Width
            QString value = values.last().remove(";").trimmed();

            QStringList parts = fullKey.split("_");
            if (parts.isEmpty()) continue;

            QTreeWidgetItem* parent = sectionItem;
            QString lastPrefix;

            QStringList cleanedKeys;
            for (const QString& part : parts) {
                if (!lastPrefix.isEmpty() && part.startsWith(lastPrefix)) {
                    cleanedKeys << part.mid(lastPrefix.length());
                } else {
                    cleanedKeys << part;
                }
                lastPrefix += part;
            }

            for (int i = 0; i < cleanedKeys.size() - 1; ++i) {
                QString key = cleanedKeys[i];
                QTreeWidgetItem* existing = nullptr;

                for (int j = 0; j < parent->childCount(); ++j) {
                    if (parent->child(j)->text(0) == key) {
                        existing = parent->child(j);
                        break;
                    }
                }

                if (existing) {
                    parent = existing;
                } else {
                    QTreeWidgetItem* node = new QTreeWidgetItem(parent);
                    node->setText(0, key);
                    parent = node;
                }
            }

            QString paramKey = cleanedKeys.last();
            QTreeWidgetItem* leaf = new QTreeWidgetItem(parent);
            leaf->setText(0, paramKey);

            QLineEdit* lineEdit = new QLineEdit(value);
            lineEdit->setFrame(false);
            lineEdit->setObjectName(fullKey);

            connect(lineEdit, &QLineEdit::returnPressed, this, [=]() {
                int r = QMessageBox::question(
                    mcfEditor, "MCF Editor",
                    "Are you sure to change this value?",
                    QMessageBox::Cancel | QMessageBox::Ok, QMessageBox::Cancel);

                if (r == QMessageBox::Cancel) {
                    lineEdit->undo();
                    return;
                }

                QString parentName = sectionItem ? sectionItem->text(0) : "";
                QString obj = lineEdit->objectName();
                QString filterString = "ID";

                if (parentName.startsWith(filterString)) {
                    parentName.remove(filterString);

                    switch(this->parent->getParameterDataType(obj, parentName.toInt())){
                    case FG_PARAM_TYPE_SIZE_T:
                    case FG_PARAM_TYPE_INT32_T:
                    case FG_PARAM_TYPE_UINT32_T:
                    case FG_PARAM_TYPE_INT64_T:
                    case FG_PARAM_TYPE_UINT64_T:{
                        this->parent->setParameterValue(obj, lineEdit->text().toInt(), parentName.toInt());
                    }break;
                    case FG_PARAM_TYPE_DOUBLE:{
                        this->parent->setParameterValue(obj, lineEdit->text().toDouble(), parentName.toInt());
                    }break;
                    case FG_PARAM_TYPE_CHAR_PTR_PTR:
                    case FG_PARAM_TYPE_CHAR_PTR:{
                        this->parent->setParameterValue(obj, lineEdit->text(), parentName.toInt());
                    }break;
                    case FG_PARAM_TYPE_STRUCT_FIELDPARAMACCESS:
                    case FG_PARAM_TYPE_STRUCT_FIELDPARAMINT:
                    case FG_PARAM_TYPE_STRUCT_FIELDPARAMINT64:
                    case FG_PARAM_TYPE_STRUCT_FIELDPARAMDOUBLE:
                    case FG_PARAM_TYPE_COMPLEX_DATATYPE:
                    case FG_PARAM_TYPE_AUTO:
                    case FG_PARAM_TYPE_INVALID:
                    default: this->parent->setParameterValue(obj, lineEdit->text().toInt(), parentName.toInt());
                        break;
                    }

                    this->refreshMCFValues();
                    if (obj.contains("FG_")) {
                        initTabWidget();
                    }
                }
            });
            widget->setItemWidget(leaf, 1, lineEdit);
        }
    }
    widget->setItemDelegate(new QStyledItemDelegate(widget));

    auto filterItems = [widget](const QString &searchText) {
        std::function<bool(QTreeWidgetItem*)> filterRecursive = [&](QTreeWidgetItem *item) -> bool {
            if (!item) return false;

            QString originalText = item->data(0, Qt::UserRole).toString();
            if (originalText.isEmpty()) {
                originalText = item->text(0);
                item->setData(0, Qt::UserRole, originalText);
            }
            item->setText(0, originalText);

            bool selfMatch = originalText.contains(searchText, Qt::CaseInsensitive);

            if (selfMatch && !searchText.isEmpty()) {
                item->setBackground(0, QBrush(Qt::yellow));
            } else {
                item->setBackground(0, QBrush());
            }

            QWidget *editor = widget->itemWidget(item, 1);
            bool editorMatch = false;
            if (editor) {
                QLineEdit *lineEdit = qobject_cast<QLineEdit*>(editor);
                if (lineEdit) {
                    editorMatch = lineEdit->text().contains(searchText, Qt::CaseInsensitive);
                }
            }

            bool childMatch = false;
            for (int j = 0; j < item->childCount(); ++j) {
                childMatch |= filterRecursive(item->child(j));
            }

            bool finalMatch = selfMatch || editorMatch || childMatch;

            item->setHidden(!finalMatch);
            item->setExpanded(finalMatch);

            if ((selfMatch || editorMatch) && item->childCount() > 0) {
                for (int j = 0; j < item->childCount(); ++j) {
                    item->child(j)->setHidden(false);
                }
            }
            return finalMatch;
        };

        widget->setUpdatesEnabled(false);
        for (int i = 0; i < widget->topLevelItemCount(); ++i) {
            filterRecursive(widget->topLevelItem(i));
        }

        widget->setUpdatesEnabled(true);
    };
    connect(lineEditSearch, &QLineEdit::textChanged, filterItems);
}


void Qylon::GrabberWidget::refreshMCFValues()
{
    if (!mcfEditor) return;

    auto treeWidget = mcfEditor->findChild<QTreeWidget*>();
    if (!treeWidget) return;

    std::function<void(QTreeWidgetItem*)> refreshItemRecursive = [&](QTreeWidgetItem* item) {
        if (!item) return;

        QWidget* widget = treeWidget->itemWidget(item, 1);
        if (widget) {
            QLineEdit* lineEdit = qobject_cast<QLineEdit*>(widget);
            if (lineEdit) {
                QString fullKey = lineEdit->objectName(); // Device1_Process0_AppletProperties_Width
                QString sectionName;
                QTreeWidgetItem* sectionItem = item;
                while (sectionItem && sectionItem->parent()) {
                    sectionItem = sectionItem->parent();
                }
                if (sectionItem) sectionName = sectionItem->text(0);

                QString filterString = "ID";
                if (sectionName.startsWith(filterString)) {
                    sectionName.remove(filterString);
                    bool ok = false;
                    int sectionIndex = sectionName.toInt(&ok);
                    if (ok) {
                        QVariant newValue;
                        auto dataType = parent->getParameterDataType(fullKey, sectionIndex);
                        switch(dataType){
                        case FG_PARAM_TYPE_SIZE_T:
                        case FG_PARAM_TYPE_INT32_T:
                        case FG_PARAM_TYPE_UINT32_T:
                        case FG_PARAM_TYPE_INT64_T:
                        case FG_PARAM_TYPE_UINT64_T:{
                            newValue = parent->getParameterIntValue(fullKey, sectionIndex);
                        }break;
                        case FG_PARAM_TYPE_DOUBLE:{
                            newValue = parent->getParameterDoubleValue(fullKey, sectionIndex);
                        }break;
                        case FG_PARAM_TYPE_CHAR_PTR_PTR:
                        case FG_PARAM_TYPE_CHAR_PTR:{
                            newValue = parent->getParameterStringValue(fullKey, sectionIndex);
                        }break;
                        case FG_PARAM_TYPE_STRUCT_FIELDPARAMACCESS:
                        case FG_PARAM_TYPE_STRUCT_FIELDPARAMINT:
                        case FG_PARAM_TYPE_STRUCT_FIELDPARAMINT64:
                        case FG_PARAM_TYPE_STRUCT_FIELDPARAMDOUBLE:
                        case FG_PARAM_TYPE_COMPLEX_DATATYPE:
                        case FG_PARAM_TYPE_AUTO:
                        case FG_PARAM_TYPE_INVALID:
                        default: newValue = parent->getParameterIntValue(fullKey, sectionIndex);
                            break;
                        }
                        QString newText = newValue.toString();
                        if (lineEdit->text() != newText) {
                            lineEdit->setText(newText);
                        }
                    }
                }
            }
        }

        for (int i = 0; i < item->childCount(); ++i) {
            refreshItemRecursive(item->child(i));
        }
    };

    for (int i = 0; i < treeWidget->topLevelItemCount(); ++i) {
        refreshItemRecursive(treeWidget->topLevelItem(i));
    }
}


#endif
