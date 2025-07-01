#ifdef PYLON_ENABLED
#include "CameraWidget.h"
#include "Camera.h"
#include "Qylon.h"
#include <QHBoxLayout>
#include <QCheckBox>
#include <QHeaderView>
#include <QSpinBox>
#include <QLineEdit>
#include <QAction>
#include <QStatusBar>
#include <QPalette>
#define NO_CAMERA "No camera found"

Qylon::CameraWidget::CameraWidget(Camera *obj) : parent(obj)
{
    setWindowTitle("Basler pylon Camera Configuration");
    setWindowIcon(QIcon(":/Resources/Icon.png"));
    list = new QComboBox;
    list->setMinimumWidth(120);
    widget = new QTreeWidget;
    widget->setHeaderLabels(QStringList() << "Feature" << "Value");

    buttonRefresh = new QToolButton(this);
    buttonRefresh->setIconSize(QSize(24,24));
    buttonRefresh->setDefaultAction(new QAction(QIcon(":/Resources/Icon/icons8-refresh-48.png"),"Refresh"));
    buttonRefresh->setAutoRaise(true);
    connect(buttonRefresh, &QToolButton::triggered, this, [=]{
        this->updateCameraList();
    });

    buttonConnect = new QToolButton(this);
    QIcon connectIcon;
    connectIcon.addFile(":/Resources/Icon/icons8-connect-48.png", QSize(24,24), QIcon::Normal, QIcon::Off);
    connectIcon.addFile(":/Resources/Icon/icons8-disconnected-48.png", QSize(24,24), QIcon::Normal, QIcon::On);
    QAction* actionConnect = new QAction("Connect", this);
    actionConnect->setCheckable(true);
    actionConnect->setIcon(connectIcon);
    connect(actionConnect, &QAction::toggled, this, [=](bool on){
        actionConnect->setText(on ? "Disconnect" : "Connect");
        if(on) this->connectCamera();
        else this->disconnectCamera();
    });
    buttonConnect->setAutoRaise(true);
    buttonConnect->setIconSize(QSize(24,24));
    buttonConnect->setDefaultAction(actionConnect);

    buttonSingleGrab = new QToolButton(this);
    buttonSingleGrab->setDefaultAction(new QAction(QIcon(":/Resources/Icon/icons8-camera-48.png"), "Single Grab"));
    buttonSingleGrab->setAutoRaise(true);
    buttonSingleGrab->setEnabled(false);
    buttonSingleGrab->setIconSize(QSize(24,24));
    connect(buttonSingleGrab, &QToolButton::triggered, this, [=]{
        this->parent->singleGrab();
    });

    buttonLiveGrab = new QToolButton(this);
    QIcon grabbingIcon;
    grabbingIcon.addFile(":/Resources/Icon/icons8-cameras-48.png", QSize(24,24),QIcon::Normal, QIcon::Off);
    grabbingIcon.addFile(":/Resources/Icon/icons8-pause-48.png", QSize(24,24),QIcon::Normal, QIcon::On);
    QAction *actionGrabbing = new QAction("Continuous Grab", this);
    actionGrabbing->setCheckable(true);
    actionGrabbing->setIcon(grabbingIcon);
    connect(actionGrabbing, &QAction::toggled, this, [=](bool on){
        actionGrabbing->setText(on ? "Stop Grabbing" : "Continuous Grab");
        if(on) this->parent->continuousGrab();
        else this->parent->stopGrab();
    });
    buttonLiveGrab->setDefaultAction(actionGrabbing);
    buttonLiveGrab->setAutoRaise(true);
    buttonLiveGrab->setEnabled(false);
    buttonLiveGrab->setIconSize(QSize(24,24));

    connect(this, &CameraWidget::grabbingState, this, [=](bool on){
        buttonSingleGrab->setEnabled(!on);
        buttonConnect->setEnabled(!on);
    });

    QHBoxLayout *camNamelayout = new QHBoxLayout;
    camNamelayout->addWidget(list);
    camNamelayout->addWidget(buttonRefresh);
    camNamelayout->setSpacing(-1);

    QHBoxLayout *buttonLayout = new QHBoxLayout;
    buttonLayout->addWidget(buttonConnect);
    buttonLayout->setSpacing(-1);
    buttonLayout->addSpacerItem(new QSpacerItem(5,5));
    buttonLayout->addWidget(buttonSingleGrab);
    buttonLayout->addWidget(buttonLiveGrab);

    QHBoxLayout *camAndButton = new QHBoxLayout;
    camAndButton->setContentsMargins(9,9,9,9);
    camAndButton->addLayout(camNamelayout);
    camAndButton->addLayout(buttonLayout);

    QVBoxLayout *tree = new QVBoxLayout;
    tree->setContentsMargins(9,0,9,0);
    tree->addWidget(widget);

    QVBoxLayout *layout = new QVBoxLayout;
    layout->setContentsMargins(0,0,0,0);
    layout->addLayout(camAndButton);
    layout->addLayout(tree);

    statusBar = new QStatusBar(this);
    statusBar->setContentsMargins(0,0,0,0);
    layout->addWidget(statusBar);
    setLayout(layout);
    updateCameraList();
}

void Qylon::CameraWidget::updateCameraList()
{
    list->clear();
    parent->getQylon()->updateCameraList();
    auto cameraList = parent->getQylon()->getCameraList();
    if(cameraList.size()==0)
        list->addItem(NO_CAMERA);
    else
        list->addItems(parent->getQylon()->getCameraList());
}

void Qylon::CameraWidget::connectCamera()
{
    if(list->currentText() == NO_CAMERA) return;

    if(parent->openCamera(list->currentText())){
        statusBar->showMessage("Connected: " + list->currentText());
    }
}

void Qylon::CameraWidget::disconnectCamera()
{
    parent->closeCamera();
    statusBar->showMessage("Disconnected: " + list->currentText());
}

void Qylon::CameraWidget::widgetGenerator(GenApi::INodeMap *nodemap)
{
    widget->clear();
    manageItems.clear();

    GenApi::NodeList_t nodes;
    nodemap->GetNodes(nodes);
    QTreeWidgetItem *cameraFeatures = new QTreeWidgetItem(widget, QStringList() << QString(this->parent->getInstantCamera()->GetDeviceInfo().GetFriendlyName()));
    manageItems.push_back(cameraFeatures);
    for(auto cat : nodes){
        if(cat->GetName() == "Root") continue;
        if(!GenApi::IsAvailable(cat)) continue;
        if(cat->GetPrincipalInterfaceType() != GenApi::EInterfaceType::intfICategory) continue;

        GenApi::NodeList_t parentsList;
        cat->GetParents(parentsList);
        if(parentsList.at(0)->GetDisplayName() == "Events Generation") continue;

        QTreeWidgetItem* item = new QTreeWidgetItem(cameraFeatures, QStringList() << cat->GetDisplayName().c_str());

        manageItems.push_back(item);
        GenApi::NodeList_t children;
        cat->GetChildren(children);

        generateChildrenWidgetItem(item, children);
    }
    widget->expandToDepth(0);
    widget->header()->resizeSection(0,200);
}

void Qylon::CameraWidget::generateChildrenWidgetItem(QTreeWidgetItem *parent, GenApi::NodeList_t children)
{
    for(auto sub : children){
        if(!GenApi::IsAvailable(sub)) continue;

        QTreeWidgetItem* subItem = new QTreeWidgetItem(parent, QStringList() << sub->GetDisplayName().c_str());
        manageItems.push_back(subItem);

        switch (sub->GetPrincipalInterfaceType()){
        case GenApi::EInterfaceType::intfIInteger:{
            if(!GenApi::IsReadable(sub)) continue;

            GenApi::CIntegerPtr ptr = sub;
            auto spinBox = new QSpinBox;
            spinBox->setEnabled(GenApi::IsWritable(ptr));
            try{
                spinBox->setAccessibleName(sub->GetName().c_str());
                spinBox->setRange(ptr->GetMin(), ptr->GetMax());
                spinBox->setValue(ptr->GetValue());
                spinBox->setSingleStep(ptr->GetInc());
                spinBox->setEnabled(GenApi::IsWritable(ptr));
            }catch (const Pylon::GenericException &e){
                statusMessage(e.GetDescription());
            }

            connect(this, &CameraWidget::nodeUpdated, spinBox, [=](){
                spinBox->blockSignals(true);
                try{
                    spinBox->setValue(ptr->GetValue());
                }catch(const Pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                }
                spinBox->blockSignals(false);
            });
            connect(this, &CameraWidget::grabbingState, spinBox, [=](bool){
                try{
                    spinBox->setEnabled(GenApi::IsWritable(ptr));
                }catch(const Pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                }
            });
            connect(spinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int value){
                try{
                    ptr->SetValue(value);
                    statusMessage(QString(ptr->GetNode()->GetDisplayName()) + " sets to " + QString::number(ptr->GetValue()));
                }catch(const Pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                }
                emit nodeUpdated();
            });
            widget->setItemWidget(subItem, parent->columnCount(), spinBox);
            break;
        }
        case GenApi::EInterfaceType::intfIFloat:{
            if(!GenApi::IsReadable(sub)) continue;

            GenApi::CFloatPtr ptr = sub;
            auto spinBox = new QDoubleSpinBox;
            spinBox->setEnabled(GenApi::IsWritable(ptr));
            try{
                spinBox->setAccessibleName(sub->GetName().c_str());
                spinBox->setDecimals(ptr->GetDisplayPrecision());
                spinBox->setRange(ptr->GetMin(), ptr->GetMax());
                spinBox->setValue(ptr->GetValue());
                // spinBox->setSingleStep(ptr->GetInc());
                spinBox->setSingleStep(0.1);
                spinBox->setEnabled(GenApi::IsWritable(ptr));
            }catch(const Pylon::GenericException &e){
                statusMessage(e.GetDescription());
            }
            connect(this, &CameraWidget::nodeUpdated, spinBox, [=](){
                spinBox->blockSignals(true);
                try{
                    spinBox->setValue(ptr->GetValue());
                }catch(const Pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                }
                spinBox->blockSignals(false);
            });
            connect(this, &CameraWidget::grabbingState, spinBox, [=](bool){
                try{
                    spinBox->setEnabled(GenApi::IsWritable(ptr));
                }catch(const Pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                }
            });
            connect(spinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=](double value){
                try{
                    ptr->SetValue(value);
                    statusMessage(QString(ptr->GetNode()->GetDisplayName()) + " sets to " + QString::number(ptr->GetValue())) ;
                    emit nodeUpdated();
                }catch(const Pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                }
            });
            widget->setItemWidget(subItem, parent->columnCount(), spinBox);
            break;
        }
        case GenApi::EInterfaceType::intfIBoolean:{
            if(!GenApi::IsReadable(sub)) continue;

            GenApi::CBooleanPtr ptr = sub;
            auto checkBox = new QCheckBox;
            checkBox->setEnabled(GenApi::IsWritable(ptr));
            try{
                checkBox->setChecked(ptr->GetValue());
                checkBox->setAccessibleName(sub->GetName().c_str());
                checkBox->setEnabled(GenApi::IsWritable(ptr));
            }catch(const Pylon::GenericException &e){
                statusMessage(e.GetDescription());
            }
            connect(this, &CameraWidget::nodeUpdated, checkBox, [=](){
                checkBox->blockSignals(true);
                try{
                    checkBox->setChecked(ptr->GetValue());
                }catch(const Pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                }
                checkBox->blockSignals(false);
            });
            connect(this, &CameraWidget::grabbingState, checkBox, [=](bool){
                try{
                    checkBox->setEnabled(GenApi::IsWritable(ptr));
                }catch(const Pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                }
            });
            connect(checkBox, &QCheckBox::clicked, this, [=](bool on){
                try{
                    ptr->SetValue(on);
                    statusMessage(QString(ptr->GetNode()->GetDisplayName()) + " is " + (ptr->GetValue() ? "On" : "Off"));
                    emit nodeUpdated();
                }catch(const Pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                }
            });
            widget->setItemWidget(subItem, parent->columnCount(), checkBox);
            break;
        }
        case GenApi::EInterfaceType::intfIString:{
            if(!GenApi::IsReadable(sub)) continue;

            GenApi::CStringPtr ptr = sub;
            QLineEdit *lineEdit = new QLineEdit;
            lineEdit->setEnabled(GenApi::IsWritable(ptr));
            lineEdit->setFrame(false);
            try{
                lineEdit->setText(ptr->GetValue().c_str());
            }catch(const Pylon::GenericException &e){
                statusMessage(e.GetDescription());
            }

            connect(this, &CameraWidget::nodeUpdated, lineEdit, [=](){
                lineEdit->blockSignals(true);
                try{
                    lineEdit->setText(ptr->GetValue().c_str());
                }catch(const Pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                }
                lineEdit->blockSignals(false);
            });
            connect(this, &CameraWidget::grabbingState, lineEdit, [=](bool){
                try{
                    lineEdit->setEnabled(GenApi::IsWritable(ptr));
                }catch(const Pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                }
            });
            connect(lineEdit, &QLineEdit::editingFinished, this, [=]{
                try{
                    ptr->SetValue(lineEdit->text().toStdString().c_str());
                    statusMessage(QString(ptr->GetNode()->GetDisplayName()) + " sets to " + ptr->GetValue());
                    emit nodeUpdated();
                }catch(const Pylon::GenericException &e){
                    qDebug() << "string 3 error";
                    statusMessage(e.GetDescription());
                }
            });
            widget->setItemWidget(subItem, parent->columnCount(), lineEdit);
            break;
        }
        case GenApi::EInterfaceType::intfIEnumeration:{
            if(!GenApi::IsReadable(sub)) continue;

            GenApi::CEnumerationPtr ptr = sub;
            auto comboBox = new QComboBox;
            comboBox->setEnabled(GenApi::IsWritable(ptr));

            try{
                Pylon::StringList_t enuList;
                ptr->GetSymbolics(enuList);

                comboBox->setAccessibleName(sub->GetName().c_str());
                comboBox->setEnabled(GenApi::IsWritable(ptr));

                for(const auto &current : enuList){
                    comboBox->addItem(QString::fromStdString(ptr->GetEntryByName(current)->GetNode()->GetDisplayName().c_str()), QVariant::fromValue((QString)current));
                }
                comboBox->setCurrentText(ptr->GetCurrentEntry()->GetNode()->GetDisplayName().c_str());
            }catch(const Pylon::GenericException &e){
                qDebug() << "Enum 1 error";
                statusMessage(e.GetDescription());
            }

            connect(this, &CameraWidget::nodeUpdated, comboBox, [=](){
                comboBox->blockSignals(true);
                try{
                    GenApi::CEnumerationPtr ptr = this->parent->getNode(comboBox->accessibleName().toStdString().c_str());
                    comboBox->setCurrentText(ptr->GetCurrentEntry()->GetNode()->GetDisplayName().c_str());
                }catch(const Pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                }
                comboBox->blockSignals(false);
            });
            connect(this, &CameraWidget::grabbingState, comboBox, [=](bool){
                try{
                    comboBox->setEnabled(GenApi::IsWritable(ptr));
                }catch(const Pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                }
            });
            connect(comboBox, &QComboBox::currentTextChanged, this, [=](QString){
                comboBox->blockSignals(true);
                try{
                    auto val = ptr->GetEntryByName(comboBox->currentData().toString().toStdString().c_str());
                    ptr->SetIntValue(val->GetNumericValue());
                    statusMessage(QString(ptr->GetNode()->GetDisplayName().c_str()) + " sets to " + val->GetSymbolic() );

                    emit nodeUpdated();
                }catch(const Pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                }
                comboBox->blockSignals(false);
            });
            widget->setItemWidget(subItem, parent->columnCount(), comboBox);
            break;
        }
        case GenApi::EInterfaceType::intfICommand:{
            GenApi::CCommandPtr ptr = sub;
            auto button = new QPushButton("Execute");
            button->setEnabled(GenApi::IsWritable(ptr));

            button->setAccessibleName(sub->GetName().c_str());
            connect(this, &CameraWidget::grabbingState, button, [=](bool){
                try{
                    button->setEnabled(GenApi::IsWritable(ptr));
                }catch(const Pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                }
            });
            connect(button, &QPushButton::clicked, this, [=](){
                try{
                    ptr->Execute();
                    statusMessage("Executed " + QString(ptr->GetNode()->GetDisplayName().c_str()));
                    emit nodeUpdated();
                }catch (const Pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                }
            });
            widget->setItemWidget(subItem, parent->columnCount(), button);

            break;
        }
        case GenApi::EInterfaceType::intfIRegister:{
            GenApi::CRegisterPtr ptr = sub;
            widget->setItemWidget(subItem, parent->columnCount(), new QLabel(QString::number(ptr->GetAddress())));
            break;
        }
        case GenApi::EInterfaceType::intfICategory:{
            GenApi::NodeList_t subChildren;
            sub->GetChildren(subChildren);
            generateChildrenWidgetItem(parent, subChildren);
            break;
        }
        case GenApi::EInterfaceType::intfIEnumEntry:
        case GenApi::EInterfaceType::intfIBase:
        case GenApi::EInterfaceType::intfIValue:
        case GenApi::EInterfaceType::intfIPort:{
            delete subItem;
            manageItems.pop_back();
            break;
        }}
    }
}

void Qylon::CameraWidget::statusMessage(QString message)
{
    Qylon::log(message);
    statusBar->showMessage(message);
}

void Qylon::CameraWidget::connectionStatus(bool connected)
{
    list->setEnabled(!connected);
    buttonRefresh->setEnabled(!connected);
    buttonSingleGrab->setEnabled(connected);

    if(connected){
        buttonLiveGrab->setEnabled(true);
        widgetGenerator(&parent->getInstantCamera()->GetNodeMap());
    }else{
        buttonConnect->defaultAction()->setChecked(false);
        buttonLiveGrab->defaultAction()->setChecked(false);
        buttonLiveGrab->setEnabled(false);
        widget->clear();

    }
}



#endif

