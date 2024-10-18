#ifdef PYLON_ENABLED
#include "vToolsWidget.h"

Qylon::vToolsWidget::vToolsWidget(vTools *parentVTools, QWidget *parent) : QWidget(parent), layout(new QVBoxLayout(this)){
    parentVTools->setResultFilter(&outputList);
    setWindowTitle("Basler vLauncher");
    setWindowIcon(QIcon(":/Resources/Icon.png"));
    Qt::WindowFlags flags = windowFlags();
    flags &= ~Qt::WindowContextHelpButtonHint;
    setWindowFlags(flags);

}

void Qylon::vToolsWidget::loadRecipeConfiguration(Pylon::DataProcessing::CRecipe *recipe){
    try{
        Pylon::StringList_t outputNames;
        recipe->GetOutputNames(outputNames);
        QGroupBox *groupBoxOutputControl = new QGroupBox;
        layoutOutputControl = new QVBoxLayout;
        groupBoxOutputControl->setLayout(layoutOutputControl);
        groupBoxOutputControl->setTitle("Output Control");
        layout->addWidget(groupBoxOutputControl);

        QHBoxLayout *layoutImage = new QHBoxLayout;
        QLabel *labelImage = new QLabel("Image Selector :");
        QComboBox *comboBoxImage = new QComboBox;
        comboBoxImage->setObjectName("Image");
        connect(comboBoxImage, &QComboBox::currentTextChanged, this, [=](){
            this->updateOutputControl();
        });
        layoutImage->addWidget(labelImage);
        layoutImage->addWidget(comboBoxImage);
        layoutOutputControl->addLayout(layoutImage);

        QHBoxLayout *layoutItemDelimiter = new QHBoxLayout;
        QLabel *labelItemDelimiter = new QLabel("Item delimiter :");
        QComboBox *comboBoxItemDelimiter = new QComboBox;
        comboBoxItemDelimiter->setObjectName("ItemDelimiter");
        comboBoxItemDelimiter->addItems(QStringList() << ";" << "|" << "/" << "&");
        comboBoxItemDelimiter->setFixedWidth(35);
        connect(comboBoxItemDelimiter, &QComboBox::currentTextChanged, this, [=](){
            this->updateOutputControl();
        });
        layoutItemDelimiter->addWidget(labelItemDelimiter);
        layoutItemDelimiter->addWidget(comboBoxItemDelimiter);
        layoutOutputControl->addLayout(layoutItemDelimiter);

        QHBoxLayout *layoutKeyValueDelimiter = new QHBoxLayout;
        QLabel *labelKeyValueDelimiter = new QLabel("Key-value delimiter :");
        QComboBox *comboBoxKeyValueDelimiter = new QComboBox;
        comboBoxKeyValueDelimiter->setObjectName("KeyValueDelimiter");
        comboBoxKeyValueDelimiter->addItems(QStringList() << "=" << ":" << ">");
        comboBoxKeyValueDelimiter->setFixedWidth(35);
        connect(comboBoxKeyValueDelimiter, &QComboBox::currentTextChanged, this, [=](){
            this->updateOutputControl();
        });
        layoutKeyValueDelimiter->addWidget(labelKeyValueDelimiter);
        layoutKeyValueDelimiter->addWidget(comboBoxKeyValueDelimiter);
        layoutOutputControl->addLayout(layoutKeyValueDelimiter);

        for(auto i = outputNames.begin(); i != outputNames.end(); ++i){
            auto val = recipe->GetOutputType(i->c_str());
            if(val == Pylon::DataProcessing::VariantDataType_PylonImage){
                comboBoxImage->blockSignals(true);
                comboBoxImage->addItem(i->c_str());
                comboBoxImage->blockSignals(false);
                outputList.currentImage = comboBoxImage->currentText();
            }else{
                QCheckBox *checkBox = new QCheckBox;
                checkBox->setText(i->c_str());
                checkBox->setChecked(true);
                connect(checkBox, &QCheckBox::clicked, this, [=](bool on){
                    this->updateOutputControl();
                });
                layoutOutputControl->addWidget(checkBox);
                outputList.items.push_back(QPair<QString, bool>(i->c_str(), true));
            }
        }
    }catch(const Pylon::GenericException &e){
        qDebug() << e.what();
    }
}

void Qylon::vToolsWidget::updateOutputControl(){
    QComboBox *comboBox = qobject_cast<QComboBox*>(sender());
    if(comboBox){
        if(comboBox->objectName() == "Image") outputList.currentImage = comboBox->currentText();
        else if(comboBox->objectName() == "ItemDelimiter") outputList.itemDelimiter = comboBox->currentText();
        else if(comboBox->objectName() == "KeyValueDelimiter") outputList.keyValueDelimiter = comboBox->currentText();
    }else{
        QCheckBox *checkBox = qobject_cast<QCheckBox*>(sender());
        if(checkBox){
            for(int i=0; i<outputList.items.size(); ++i){
                auto currentValue = outputList.items.at(i);
                if(currentValue.first == checkBox->text()){
                    outputList.items[i].second = checkBox->isChecked();
                }
            }
        }
    }
}
#endif
