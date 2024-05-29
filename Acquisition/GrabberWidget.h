#ifndef GRABBERWIDGET_H
#define GRABBERWIDGET_H
#ifdef GRABBER_ENABLED
#include <QObject>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QDir>
#include <QFileDialog>
#include <QLabel>
#include <QSpacerItem>
#include <QGroupBox>

#include "Grabber.h"


namespace Qylon{
class GrabberWidget : public QWidget
{
    Q_OBJECT
public:
    GrabberWidget(Grabber *obj);
    void setDefaultAppletPath(QString path);
    void setDefaultConfigPath(QString path);
    void setDMACount(int val);

signals:
    void changedDMACount(int val);

private:
    Grabber *parent;
    QVBoxLayout *layout;

    QLineEdit *lineApplet;
    QLineEdit *lineConfig;

    QString defaultAppletPath = "";
    QString defaultConfigPath = "";
};
}
#endif
#endif // GRABBERWIDGET_H
