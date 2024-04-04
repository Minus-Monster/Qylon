#ifndef CAMERAWIDGET_H
#define CAMERAWIDGET_H
#ifdef PYLON_ENABLED
#include <QObject>
#include <QWidget>
#include <QTreeWidget>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QSpinBox>

#include "Camera.h"


namespace Qylon{
class CameraWidget : public QWidget
{
    Q_OBJECT
public:
    CameraWidget(Camera *obj);

    void updateCameraList();

    void widgetGenerator(GenApi_3_1_Basler_pylon::INodeMap *nodemap);
    void generateChildrenWidgetItem(QTreeWidgetItem *parent, GenApi_3_1_Basler_pylon::NodeList_t children);
    void generateChildrenWidgetItem(QTreeWidget *parent, GenApi_3_1_Basler_pylon::NodeList_t children);
    void statusMessage(QString message);
    QLabel *status;

public slots:
    void connectCamera();
    void disconnectCamera();
    void connectedCameraFromOutside();

signals:
    void nodeUpdated();
    void grabbingState(bool isGrabbing);

private:
    Camera *parent;
    QTreeWidget *widget;
    QList<QTreeWidgetItem*> manageItems;
    QComboBox *list;

    QLabel *name_tag;
    QPushButton *refreshButton;
    QPushButton *connectButton;
    QPushButton *disconnectButton;
};
}

#endif
#endif // CAMERAWIDGET_H
