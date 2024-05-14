#include "SerialCommunicationWidget.h"
#include "SerialCommunication.h"
#include <QResizeEvent>

Qylon::SerialCommunicationWidget::SerialCommunicationWidget(SerialCommunication *parent) : serial(parent){
    setWindowTitle(" ");
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    this->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    comboBoxSerialList = new QComboBox;
    buttonRefresh = new QPushButton("Refresh");
    connect(buttonRefresh, &QPushButton::clicked, this, [&](){
        this->initialize();
    });
    QHBoxLayout *listLayout = new QHBoxLayout;
    layout->addLayout(listLayout);

    listLayout->addWidget(comboBoxSerialList, 8);
    listLayout->addWidget(buttonRefresh, 2);

    labelDescription = new QLabel("Description : ");
    labelManufacturer = new QLabel("Manufacturer : ");
    labelSerialNumber = new QLabel("Serial Number : ");
    labelVendorIdentifier = new QLabel("Vendor Identifier : ");
    labelProductIdentifier = new QLabel("Product Identifier : " );
    labelSystemLocation = new QLabel("System Location : ");
    labelConnectionStatus = new QLabel("Connection : Free");

    comboBoxBaudRate = new QComboBox;
    comboBoxBaudRate->addItem("1200", QSerialPort::BaudRate::Baud1200);
    comboBoxBaudRate->addItem("2400", QSerialPort::BaudRate::Baud2400);
    comboBoxBaudRate->addItem("4800", QSerialPort::BaudRate::Baud4800);
    comboBoxBaudRate->addItem("9600", QSerialPort::BaudRate::Baud9600);
    comboBoxBaudRate->addItem("19200", QSerialPort::BaudRate::Baud19200);
    comboBoxBaudRate->addItem("38400", QSerialPort::BaudRate::Baud38400);
    comboBoxBaudRate->addItem("57600", QSerialPort::BaudRate::Baud57600);
    comboBoxBaudRate->addItem("115200", QSerialPort::BaudRate::Baud115200);
    comboBoxBaudRate->setCurrentText("115200");

    comboBoxDataBits = new QComboBox;
    comboBoxDataBits->addItem("5", QSerialPort::DataBits::Data5);
    comboBoxDataBits->addItem("6", QSerialPort::DataBits::Data6);
    comboBoxDataBits->addItem("7", QSerialPort::DataBits::Data7);
    comboBoxDataBits->addItem("8", QSerialPort::DataBits::Data8);
    comboBoxDataBits->setCurrentText("8");

    comboBoxParity = new QComboBox;
    comboBoxParity->addItem("NoParity", QSerialPort::Parity::NoParity);
    comboBoxParity->addItem("Even", QSerialPort::Parity::EvenParity);
    comboBoxParity->addItem("Odd", QSerialPort::Parity::OddParity);
    comboBoxParity->addItem("Mark", QSerialPort::Parity::MarkParity);
    comboBoxParity->addItem("Space", QSerialPort::Parity::SpaceParity);
    comboBoxParity->setCurrentText("NoParity");

    comboBoxStopBits = new QComboBox;
    comboBoxStopBits->addItem("1", QSerialPort::StopBits::OneStop);
    comboBoxStopBits->addItem("1.5", QSerialPort::StopBits::OneAndHalfStop);
    comboBoxStopBits->addItem("2", QSerialPort::StopBits::TwoStop);
    comboBoxStopBits->setCurrentText("1");

    QGroupBox *groupBoxConnection = new QGroupBox;
    QVBoxLayout *groupBoxLayout = new QVBoxLayout;
    groupBoxConnection->setLayout(groupBoxLayout);

    QHBoxLayout *baudLayout = new QHBoxLayout;
    baudLayout->addWidget(new QLabel("Baud rate : "));
    baudLayout->addWidget(comboBoxBaudRate);
    groupBoxLayout->addLayout(baudLayout);

    QHBoxLayout *dataBitsLayout = new QHBoxLayout;
    dataBitsLayout->addWidget(new QLabel("Data bits : "));
    dataBitsLayout->addWidget(comboBoxDataBits);
    groupBoxLayout->addLayout(dataBitsLayout);

    QHBoxLayout *stopBitsLayout = new QHBoxLayout;
    stopBitsLayout->addWidget(new QLabel("Stop bits : "));
    stopBitsLayout->addWidget(comboBoxStopBits);
    groupBoxLayout->addLayout(stopBitsLayout);

    QHBoxLayout *parityLayout = new QHBoxLayout;
    parityLayout->addWidget(new QLabel("Parity : "));
    parityLayout->addWidget(comboBoxParity);
    groupBoxLayout->addLayout(parityLayout);

    buttonConnect = new QPushButton("Connect");
    connect(buttonConnect, &QPushButton::clicked, this, [&](){
        auto val = this->serial->connect(comboBoxSerialList->currentText(), QSerialPort::BaudRate(comboBoxBaudRate->currentData().toInt()), QSerialPort::DataBits(comboBoxDataBits->currentData().toInt()),
                              QSerialPort::Parity(comboBoxParity->currentData().toInt()),QSerialPort::StopBits(comboBoxStopBits->currentData().toInt()));
        if(val){
            buttonDisconnect->setEnabled(true);
            buttonConnect->setEnabled(false);
            buttonRefresh->setEnabled(false);
            comboBoxSerialList->setEnabled(false);
            lineEditReceivedMessage->setVisible(true);
            buttonSendTestMessage->setVisible(true);
            updateStatus();
        }
    });
    buttonDisconnect = new QPushButton("Disconnect");
    buttonDisconnect->setEnabled(false);
    connect(buttonDisconnect, &QPushButton::clicked, this, [&](){
        this->serial->disconnect();
        buttonDisconnect->setEnabled(false);
        buttonConnect->setEnabled(true);
        buttonRefresh->setEnabled(true);
        comboBoxSerialList->setEnabled(true);
        lineEditReceivedMessage->setVisible(false);
        buttonSendTestMessage->setVisible(false);
        updateStatus();
    });
    QHBoxLayout *hLayout = new QHBoxLayout;
    hLayout->addWidget(buttonConnect);
    hLayout->addWidget(buttonDisconnect);
    groupBoxLayout->addItem(new QSpacerItem(10, 20));
    groupBoxLayout->addLayout(hLayout);

    QVBoxLayout *layoutAfterConnection = new QVBoxLayout;
    lineEditReceivedMessage = new QLineEdit;
    buttonSendTestMessage = new QPushButton("Send a test message");
    layoutAfterConnection->addWidget(lineEditReceivedMessage);
    layoutAfterConnection->addWidget(buttonSendTestMessage);
    groupBoxLayout->addLayout(layoutAfterConnection);
    lineEditReceivedMessage->setVisible(false);
    lineEditReceivedMessage->setPlaceholderText("Received : ");
    lineEditReceivedMessage->setReadOnly(true);
    connect(this->serial, &SerialCommunication::receivedData, lineEditReceivedMessage, [this](QString data){
        lineEditReceivedMessage->clear();
        lineEditReceivedMessage->setText(data);
    });
    buttonSendTestMessage->setVisible(false);
    connect(buttonSendTestMessage, &QPushButton::clicked, this, [&](){
        this->serial->sendData("Hello World!");
    });

    layout->addWidget(labelDescription);
    layout->addWidget(labelManufacturer);
    layout->addWidget(labelSerialNumber);
    layout->addWidget(labelVendorIdentifier);
    layout->addWidget(labelProductIdentifier);
    layout->addWidget(labelSystemLocation);
    layout->addWidget(labelConnectionStatus);
    layout->addWidget(groupBoxConnection);
    layout->addStretch();

    connect(comboBoxSerialList, &QComboBox::currentTextChanged, this, &SerialCommunicationWidget::updateStatus);
    initialize();
}

void Qylon::SerialCommunicationWidget::initialize(){
    comboBoxSerialList->clear();
    foreach(const QSerialPortInfo &port, QSerialPortInfo::availablePorts()) {
        comboBoxSerialList->addItem(port.portName());
    }
}

void Qylon::SerialCommunicationWidget::updateStatus(){
    foreach(const QSerialPortInfo &port, QSerialPortInfo::availablePorts()) {
        if(comboBoxSerialList->currentText() == port.portName()){
            labelDescription->setText("Description : " + port.description());
            labelManufacturer->setText("Manufacturer : " + port.manufacturer());
            labelSerialNumber->setText("Serial Number : " + port.serialNumber());
            labelVendorIdentifier->setText("Vendor Identifier : " + QString::number(port.vendorIdentifier()));
            labelProductIdentifier->setText("Product Identifier : " + QString::number(port.productIdentifier()));
            labelSystemLocation->setText("System Location : " + port.systemLocation());
            labelConnectionStatus->setText("Connection : " + QString(port.isBusy() ? "Busy" : "Free"));
        }
    }
}
