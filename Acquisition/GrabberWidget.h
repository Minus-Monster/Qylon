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
#include <QTabWidget>
#include "Grabber.h"


namespace Qylon{
class GrabberWidget : public QWidget
{
    Q_OBJECT
public:
    GrabberWidget(Grabber *obj);
    void getMCFStructure(QString mcfPath);
    void saveMCFStructure(QString savePath); //Legacy
    void initTabWidget();
    void refreshMCFValues();

private:
    Grabber *parent;
    QVBoxLayout *layout;
    QTabWidget *tabWidgetDMA;

    QLineEdit *lineLoadApplet;
    QLineEdit *lineLoadConfig;

    QString defaultAppletPath = "";
    QString defaultConfigPath = "";
    QDialog *mcfEditor = nullptr;
};
}
#endif
#endif // GRABBERWIDGET_H
