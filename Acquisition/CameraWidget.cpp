#include "CameraWidget.h"
#include "Camera.h"
#include "Qylon.h"
#include <QHBoxLayout>
#include <QCheckBox>
#include <QHeaderView>
#include <QSpinBox>
#include <QLineEdit>

Qylon::CameraWidget::CameraWidget(Camera *obj) : parent(obj)
{
    setWindowTitle("Basler pylon Camera Configuration");
    setWindowIcon(QIcon(":/Qylon/Resources/Icon.png"));
    list = new QComboBox;
    list->setMinimumWidth(120);
    widget = new QTreeWidget;
    widget->setHeaderLabels(QStringList() << "Features" << "Value");

    name_tag = new QLabel("Camera : ");
    status = new QLabel("Camera Status : ");
    refreshButton = new QPushButton(QIcon(":/Resources/Icon/icons8-refresh-48.png"),"");
    refreshButton->setFlat(true);
    refreshButton->setToolTip("Refresh");
    connect(refreshButton, &QPushButton::clicked, this, [=](bool){
        this->updateCameraList();
    });
    connectButton = new QPushButton(QIcon(":/Resources/Icon/icons8-connect-48.png"),"");
    connectButton->setFlat(true);
    connectButton->setToolTip("Connect");
    connect(connectButton, &QPushButton::clicked, this, [=](bool){
        this->connectCamera();
    });

    disconnectButton = new QPushButton(QIcon(":/Resources/Icon/icons8-disconnected-48.png"),"");
    disconnectButton->setFlat(true);
    disconnectButton->setToolTip("Disconnect");
    disconnectButton->setEnabled(false);
    connect(disconnectButton, &QPushButton::clicked, this, [=](bool){
        this->disconnectCamera();
    });

    QHBoxLayout *camNamelayout = new QHBoxLayout;
    camNamelayout->addWidget(name_tag);
    camNamelayout->addWidget(list);
    camNamelayout->addWidget(refreshButton);
    camNamelayout->setSpacing(-1);

    QHBoxLayout *buttonLayout = new QHBoxLayout;
    buttonLayout->addWidget(connectButton);
    buttonLayout->addWidget(disconnectButton);
    buttonLayout->setSpacing(-1);

    QHBoxLayout *camAndButton = new QHBoxLayout;
    camAndButton->addLayout(camNamelayout);
    camAndButton->addStretch(100);
    camAndButton->addLayout(buttonLayout);


    QVBoxLayout *treeAndStatus = new QVBoxLayout;
    treeAndStatus->addWidget(status);
    //    QLineEdit *search = new QLineEdit;
    //    search->setClearButtonEnabled(true);
    //    treeAndStatus->addWidget(search);
    treeAndStatus->addWidget(widget);


    QVBoxLayout *layout = new QVBoxLayout;
    layout->addLayout(camAndButton);
    layout->addLayout(treeAndStatus);

    setLayout(layout);
    updateCameraList();
}

void Qylon::CameraWidget::updateCameraList()
{
    list->clear();
    parent->getQylon()->updateCameraList();
    list->addItems(parent->getQylon()->getCameraList());
}

void Qylon::CameraWidget::connectCamera()
{
    if(parent->openCamera(list->currentText())){
        list->setEnabled(false);
        connectButton->setEnabled(false);
        refreshButton->setEnabled(false);
        disconnectButton->setEnabled(true);

        //        updateTreeWidget();
        widgetGenerator(&parent->getInstantCamera()->GetNodeMap());
        status->setText("Camera Status : connected the camera");
    }
}

void Qylon::CameraWidget::disconnectCamera()
{
    list->setEnabled(true);
    connectButton->setEnabled(true);
    refreshButton->setEnabled(true);
    disconnectButton->setEnabled(false);
    parent->closeCamera();
    widget->clear();
    status->setText("Camera Status : disconnected the camera");
}

void Qylon::CameraWidget::connectedCameraFromOutside()
{
    list->setEnabled(false);
    connectButton->setEnabled(false);
    refreshButton->setEnabled(false);
    disconnectButton->setEnabled(true);
    widgetGenerator(&this->parent->getInstantCamera()->GetNodeMap());
    status->setText("Camera Status : connected the camera");
}

void Qylon::CameraWidget::widgetGenerator(GenApi_3_1_Basler_pylon::INodeMap *nodemap)
{
    widget->clear();
    manageItems.clear();

    GenApi_3_1_Basler_pylon::NodeList_t nodes;
    nodemap->GetNodes(nodes);
    QTreeWidgetItem *cameraFeatures = new QTreeWidgetItem(widget, QStringList() << QString(this->parent->getInstantCamera()->GetDeviceInfo().GetFriendlyName()));
    manageItems.push_back(cameraFeatures);
    for(auto cat : nodes){
        if(!cat->IsFeature()) continue;
        if(cat->GetName() == "Root") continue;
        if(!GenApi_3_1_Basler_pylon::IsAvailable(cat)) continue;
        if(cat->GetPrincipalInterfaceType() != GenApi_3_1_Basler_pylon::EInterfaceType::intfICategory) continue;

        GenApi_3_1_Basler_pylon::NodeList_t parentsList;
        cat->GetParents(parentsList);
        if(parentsList.at(0)->GetDisplayName() == "Events Generation") continue;

        QTreeWidgetItem* item = new QTreeWidgetItem(cameraFeatures, QStringList() << cat->GetDisplayName().c_str());

        manageItems.push_back(item);
        GenApi_3_1_Basler_pylon::NodeList_t children;
        cat->GetChildren(children);

        generateChildrenWidgetItem(item, children);
    }
    widget->expandToDepth(0);
    widget->header()->resizeSection(0,200);
}

void Qylon::CameraWidget::generateChildrenWidgetItem(QTreeWidgetItem *parent, GenApi_3_1_Basler_pylon::NodeList_t children)
{
    for(auto sub : children){
        if(!sub->IsFeature() ) continue;
        if(!GenApi_3_1_Basler_pylon::IsAvailable(sub)) continue;

        QTreeWidgetItem* subItem = new QTreeWidgetItem(parent, QStringList() << sub->GetDisplayName().c_str());
        manageItems.push_back(subItem);
        switch (sub->GetPrincipalInterfaceType()){
        case GenApi_3_1_Basler_pylon::EInterfaceType::intfIInteger:{
            if(!GenApi_3_1_Basler_pylon::IsReadable(sub)) continue;
            auto spinBox = new QSpinBox;
            try{
                GenApi_3_1_Basler_pylon::CIntegerPtr ptr = sub->GetNodeMap()->GetNode(sub->GetName());
                spinBox->setAccessibleName(sub->GetName().c_str());
                spinBox->setRange(ptr->GetMin(), ptr->GetMax());
                spinBox->setValue(ptr->GetValue());
                spinBox->setSingleStep(ptr->GetInc());
                spinBox->setEnabled(GenApi_3_1_Basler_pylon::IsWritable(ptr));
            }catch (const GenICam_3_1_Basler_pylon::GenericException &e){
                statusMessage(e.GetDescription());
                qDebug() << e.what();
            }
            connect(this, &CameraWidget::nodeUpdated, spinBox, [=](){
                spinBox->blockSignals(true);
                try{
                    GenApi_3_1_Basler_pylon::CIntegerPtr ptr = this->parent->getNodemap(spinBox->accessibleName().toStdString().c_str());
                    spinBox->setValue(ptr->GetValue());
                }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                    qDebug() << e.what();
                }
                spinBox->blockSignals(false);
            });
            connect(this, &CameraWidget::grabbingState, spinBox, [=](bool){
                try{
                    GenApi_3_1_Basler_pylon::CIntegerPtr ptr = this->parent->getNodemap(spinBox->accessibleName().toStdString().c_str());
                    spinBox->setEnabled(GenApi_3_1_Basler_pylon::IsWritable(ptr));
                }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                    qDebug() << e.what();
                }
            });
            connect(spinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int value){
                try{
                    qDebug() << "Exp?" << value;
                    GenApi_3_1_Basler_pylon::CIntegerPtr ptr = this->parent->getNodemap(spinBox->accessibleName().toStdString().c_str());
                    ptr->SetValue(value);
                    statusMessage(QString(ptr->GetNode()->GetDisplayName()) + " is changed to " + QString::number(ptr->GetValue())) ;
                    //                    auto val = this->parent->setNodeValue(spinBox->accessibleName(), value);
                    //                    statusMessage(QString(val->GetNode()->GetDisplayName()) + " is changed to " + QString::number(val->GetValue())) ;
                    emit nodeUpdated();
                }catch (const GenICam_3_1_Basler_pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                    qDebug() << e.what();
                }
            });
            widget->setItemWidget(subItem, parent->columnCount(), spinBox);
            break;
        }
        case GenApi_3_1_Basler_pylon::EInterfaceType::intfIFloat:{
            auto spinBox = new QDoubleSpinBox;
            try{
                GenApi_3_1_Basler_pylon::CFloatPtr ptr = sub->GetNodeMap()->GetNode(sub->GetName());
                spinBox->setAccessibleName(sub->GetName().c_str());
                spinBox->setDecimals(ptr->GetDisplayPrecision());
                spinBox->setEnabled(GenApi_3_1_Basler_pylon::IsWritable(ptr));
                spinBox->setRange(ptr->GetMin(), ptr->GetMax());
                spinBox->setValue(ptr->GetValue());
                spinBox->setSingleStep(0.1);
            }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
                statusMessage(e.GetDescription());
                qDebug() << e.what();

            }
            connect(this, &CameraWidget::nodeUpdated, spinBox, [=](){
                spinBox->blockSignals(true);
                try{
                    GenApi_3_1_Basler_pylon::CFloatPtr ptr = this->parent->getNodemap(spinBox->accessibleName().toStdString().c_str());
                    spinBox->setValue(ptr->GetValue());
                }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                    qDebug() << e.what();
                }
                spinBox->blockSignals(false);
            });
            connect(this, &CameraWidget::grabbingState, spinBox, [=](bool){
                try{
                    GenApi_3_1_Basler_pylon::CFloatPtr ptr = this->parent->getNodemap(spinBox->accessibleName().toStdString().c_str());
                    spinBox->setEnabled(GenApi_3_1_Basler_pylon::IsWritable(ptr));
                }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                    qDebug() << e.what();
                }
            });
            connect(spinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [=](double value){
                try{
                    GenApi_3_1_Basler_pylon::CFloatPtr ptr = this->parent->getNodemap(spinBox->accessibleName().toStdString().c_str());
                    //                    if(!GenApi_3_1_Basler_pylon::IsWritable(ptr)) statusMessage("Couldn't apply the value due to deny of writing");
                    ptr->SetValue(value);
                    statusMessage(QString(ptr->GetNode()->GetDisplayName()) + " is changed to " + QString::number(ptr->GetValue())) ;
                    emit nodeUpdated();
                }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                    qDebug() << e.what();

                }
            });
            widget->setItemWidget(subItem, parent->columnCount(), spinBox);
            break;
        }
        case GenApi_3_1_Basler_pylon::EInterfaceType::intfIBoolean:{
            auto checkBox = new QCheckBox;
            try{
                GenApi_3_1_Basler_pylon::CBooleanPtr ptr = sub->GetNodeMap()->GetNode(sub->GetName());
                checkBox->setChecked(ptr->GetValue());
                checkBox->setAccessibleName(sub->GetName().c_str());
                checkBox->setEnabled(GenApi_3_1_Basler_pylon::IsWritable(ptr));
            }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
                statusMessage(e.GetDescription());
                qDebug() << e.what();
            }
            connect(this, &CameraWidget::nodeUpdated, checkBox, [=](){
                checkBox->blockSignals(true);
                try{
                    GenApi_3_1_Basler_pylon::CBooleanPtr ptr = this->parent->getNodemap(checkBox->accessibleName().toStdString().c_str());
                    checkBox->setChecked(ptr->GetValue());
                }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                    qDebug() << e.what();
                }
                checkBox->blockSignals(false);
            });
            connect(this, &CameraWidget::grabbingState, checkBox, [=](bool){
                try{
                    GenApi_3_1_Basler_pylon::CBooleanPtr ptr = this->parent->getNodemap(checkBox->accessibleName().toStdString().c_str());
                    checkBox->setEnabled(GenApi_3_1_Basler_pylon::IsWritable(ptr));
                }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                    qDebug() << e.what();
                }
            });
            connect(checkBox, &QCheckBox::clicked, this, [=](bool on){
                try{
                    GenApi_3_1_Basler_pylon::CBooleanPtr ptr = this->parent->getNodemap(checkBox->accessibleName().toStdString().c_str());
                    ptr->SetValue(on);
                    statusMessage(QString(ptr->GetNode()->GetDisplayName()) + " is " + (ptr->GetValue() ? "On" : "Off"));
                    emit nodeUpdated();
                }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                    qDebug() << e.what();
                }
            });
            widget->setItemWidget(subItem, parent->columnCount(), checkBox);
            break;
        }
        case GenApi_3_1_Basler_pylon::EInterfaceType::intfIString:{
            /// If you want to edit nodes in this side, you need modify this function to make to write nodes
            GenApi_3_1_Basler_pylon::CStringPtr ptr = sub->GetNodeMap()->GetNode(sub->GetName());
            QLineEdit *lineEdit = new QLineEdit;
            lineEdit->setEnabled(GenApi_3_1_Basler_pylon::IsWritable(ptr));
            lineEdit->setFrame(false);
            lineEdit->setText(ptr->GetValue().c_str());
            widget->setItemWidget(subItem, parent->columnCount(), lineEdit);
            break;
        }
        case GenApi_3_1_Basler_pylon::EInterfaceType::intfIEnumeration:{
            auto comboBox = new QComboBox;
            GenApi_3_1_Basler_pylon::CEnumerationPtr ptr = sub->GetNodeMap()->GetNode(sub->GetName());;
            Pylon::StringList_t enuList;
            ptr->GetSymbolics(enuList);

            if(sub->GetName() == "TriggerMode"){
            }

            comboBox->setAccessibleName(sub->GetName().c_str());
            comboBox->setEnabled(GenApi_3_1_Basler_pylon::IsWritable(ptr));

            for(const auto &current : enuList){
                comboBox->addItem(QString::fromStdString(ptr->GetEntryByName(current)->GetNode()->GetDisplayName().c_str()), QVariant::fromValue((QString)current));
            }
            comboBox->setCurrentText(ptr->GetCurrentEntry()->GetNode()->GetDisplayName().c_str());
            connect(this, &CameraWidget::nodeUpdated, comboBox, [=](){
                comboBox->blockSignals(true);
                try{
                    GenApi_3_1_Basler_pylon::CEnumerationPtr ptr = this->parent->getNodemap(comboBox->accessibleName().toStdString().c_str());
                    comboBox->setCurrentText(ptr->GetCurrentEntry()->GetNode()->GetDisplayName().c_str());
                }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                    qDebug() << e.what();
                }
                comboBox->blockSignals(false);
            });
            connect(this, &CameraWidget::grabbingState, comboBox, [=](bool){
                try{
                    GenApi_3_1_Basler_pylon::CEnumerationPtr ptr = this->parent->getNodemap(comboBox->accessibleName().toStdString().c_str());
                    comboBox->setEnabled(GenApi_3_1_Basler_pylon::IsWritable(ptr));
                }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                    qDebug() << e.what();
                }
            });
            connect(comboBox, &QComboBox::currentTextChanged, this, [=](QString){
                comboBox->blockSignals(true);
                try{
                    GenApi_3_1_Basler_pylon::CEnumerationPtr ptr = this->parent->getNodemap(comboBox->accessibleName().toStdString().c_str());
                    auto val = ptr->GetEntryByName(comboBox->currentData().toString().toStdString().c_str());
                    ptr->SetIntValue(val->GetNumericValue());
                    statusMessage(QString(ptr->GetNode()->GetDisplayName().c_str()) + " is changed to " + val->GetSymbolic() );
                    emit nodeUpdated();
                }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                    qDebug() << e.what();
                }
                comboBox->blockSignals(false);
            });
            widget->setItemWidget(subItem, parent->columnCount(), comboBox);
            break;
        }
        case GenApi_3_1_Basler_pylon::EInterfaceType::intfICommand:{
            auto button = new QPushButton("Execute");
            GenApi_3_1_Basler_pylon::CCommandPtr ptr = sub->GetNodeMap()->GetNode(sub->GetName());;
            button->setEnabled(GenApi_3_1_Basler_pylon::IsWritable(ptr));
            button->setAccessibleName(sub->GetName().c_str());
            connect(this, &CameraWidget::grabbingState, button, [=](bool){
                try{
                    GenApi_3_1_Basler_pylon::CCommandPtr ptr = this->parent->getNodemap(button->accessibleName().toStdString().c_str());
                    button->setEnabled(GenApi_3_1_Basler_pylon::IsWritable(ptr));
                }catch(const GenICam_3_1_Basler_pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                    qDebug() << e.what();
                }
            });
            connect(button, &QPushButton::clicked, [=](){
                try{
                    GenApi_3_1_Basler_pylon::CCommandPtr ptr = sub->GetNodeMap()->GetNode(button->accessibleName().toStdString().c_str());
                    ptr->Execute();
                    statusMessage(QString(ptr->GetNode()->GetDisplayName().c_str()) + " is executed");
                    emit nodeUpdated();
                }catch (const GenICam_3_1_Basler_pylon::GenericException &e){
                    statusMessage(e.GetDescription());
                }
            });
            widget->setItemWidget(subItem, parent->columnCount(), button);

            break;
        }
        case GenApi_3_1_Basler_pylon::EInterfaceType::intfIRegister:{
            GenApi_3_1_Basler_pylon::CRegisterPtr ptr = sub->GetNodeMap()->GetNode(sub->GetName());;
            widget->setItemWidget(subItem, parent->columnCount(), new QLabel(QString::number(ptr->GetAddress())));
            break;
        }
        case GenApi_3_1_Basler_pylon::EInterfaceType::intfICategory:{
            GenApi_3_1_Basler_pylon::NodeList_t subChildren;
            sub->GetChildren(subChildren);

            generateChildrenWidgetItem(parent, subChildren);
            break;
        }
        case GenApi_3_1_Basler_pylon::EInterfaceType::intfIEnumEntry:
        case GenApi_3_1_Basler_pylon::EInterfaceType::intfIBase:
        case GenApi_3_1_Basler_pylon::EInterfaceType::intfIValue:
        case GenApi_3_1_Basler_pylon::EInterfaceType::intfIPort:{
            delete subItem;
            manageItems.pop_back();
            break;
        }}
    }
}

void Qylon::CameraWidget::statusMessage(QString message)
{
    Qylon::log(message);
    status->setText("Camera Status : " + message);
}



