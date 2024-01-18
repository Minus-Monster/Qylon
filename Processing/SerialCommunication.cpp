#include "SerialCommunication.h"
#include "SerialCommunicationWidget.h"

Qylon::SerialCommunication::SerialCommunication() : widget(new Qylon::SerialCommunicationWidget(this)){}
bool Qylon::SerialCommunication::connect(QString portName, BaudRate baudRate, DataBits dataBits, Parity parity, StopBits stopBits){
    setPortName(portName);
    setBaudRate(baudRate);
    setDataBits(dataBits);
    setParity(parity);
    setStopBits(stopBits);

    if(open(QIODevice::ReadWrite)){
        qDebug() << "Serial port connected.";
        QObject::connect(this, &SerialCommunication::readyRead, [&](){
            auto read = this->readAll();
            qDebug() << "[Serial comm received data] : " << read;
            emit this->receivedData(read);
        });
    }else{
        qDebug() << "Connection failed.\n";
        return false;
    }
    return true;
}

void Qylon::SerialCommunication::disconnect(){
    this->close();
}

bool Qylon::SerialCommunication::sendData(QString data){
    auto d = write(data.toStdString().c_str());
    if(d==-1) return false;
    else if(d != data.size()){
        return false;
    }

    return true;
}
