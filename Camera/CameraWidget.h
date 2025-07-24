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
#include <QToolButton>
#include <QStatusBar>
#include "Camera.h"


namespace Qylon{
class CameraWidget : public QWidget
{
    Q_OBJECT
public:
    CameraWidget(Camera *obj);

    void updateCameraList();
    void widgetGenerator(GenApi::INodeMap *nodemap);
    void generateChildrenWidgetItem(QTreeWidgetItem *parent, GenApi::NodeList_t children);
    // void generateChildrenWidgetItem(QTreeWidget *parent, GenApi::NodeList_t children);
    void statusMessage(QString message);
    QAction* getSingleGrabAction(){ return actionSingleGrab; }
    QAction* getContinuousGrabAction(){ return actionLiveGrab; }

public slots:
    void connectionStatus(bool connected);
    bool connectCamera();
    void disconnectCamera();

signals:
    void nodeUpdated();
    void grabbingState(bool isGrabbing);

private:
    Camera *camera;
    QTreeWidget *widget;
    QList<QTreeWidgetItem*> manageItems;
    QComboBox *list;

    QStatusBar *statusBar;
    QToolButton *buttonRefresh;
    QToolButton *buttonConnect;
    QToolButton *buttonSingleGrab;
    QToolButton *buttonLiveGrab;
    QAction *actionSingleGrab;
    QAction *actionLiveGrab;


};
}

#endif
#endif // CAMERAWIDGET_H
