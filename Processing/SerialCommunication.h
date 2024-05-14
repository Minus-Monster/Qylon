#ifndef SERIALCOMMUNICATION_H
#define SERIALCOMMUNICATION_H

#include <QDebug>
#include <QObject>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

namespace Qylon{
class SerialCommunicationWidget;
class SerialCommunication : public QSerialPort{
    Q_OBJECT
public:
    SerialCommunication();
    bool connect(QString portName, QSerialPort::BaudRate baudRate=QSerialPort::BaudRate::Baud9600, QSerialPort::DataBits dataBits=QSerialPort::DataBits::Data8,
                 QSerialPort::Parity parity=QSerialPort::Parity::NoParity, QSerialPort::StopBits stopBits =QSerialPort::StopBits::OneStop);
    void disconnect();
    bool sendData(QString data);
    QWidget* getWidget(){
        return reinterpret_cast<QWidget*>(widget);
    }


signals:
    void receivedData(QString data);

private:
    SerialCommunicationWidget *widget;
};
}

#endif // SERIALCOMMUNICATION_H
