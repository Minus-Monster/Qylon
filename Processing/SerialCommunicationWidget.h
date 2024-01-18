#ifndef SERIALCOMMUNICATIONWIDGET_H
#define SERIALCOMMUNICATIONWIDGET_H

#include <QWidget>
#include <QObject>
#include <QDebug>
#include <QWidget>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QSpacerItem>
#include <QLineEdit>

namespace Qylon{
class SerialCommunication;
class SerialCommunicationWidget : public QWidget
{
    Q_OBJECT
public:
    SerialCommunicationWidget(SerialCommunication* parent = nullptr);
    void initialize();
public slots:
    void updateStatus();

private:
    QComboBox *comboBoxSerialList;
    QLabel *labelDescription;
    QLabel *labelManufacturer;
    QLabel *labelSerialNumber;
    QLabel *labelSystemLocation;
    QLabel *labelVendorIdentifier;
    QLabel *labelProductIdentifier;
    QLabel *labelConnectionStatus;

    QPushButton *buttonConnect;
    QPushButton *buttonDisconnect;

    QComboBox *comboBoxBaudRate;
    QComboBox *comboBoxDataBits;
    QComboBox *comboBoxParity;
    QComboBox *comboBoxStopBits;

    QLineEdit *lineEditReceivedMessage;
    QPushButton *buttonSendTestMessage;

    SerialCommunication *serial;
};
}
#endif // SERIALCOMMUNICATIONWIDGET_H
