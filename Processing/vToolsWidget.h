#ifndef VTOOLSWIDGET_H
#define VTOOLSWIDGET_H

#include <QWidget>
#include <QDebug>
#include <QVBoxLayout>
#include <QComboBox>
#include <QLabel>
#include <QGroupBox>
#include <QCheckBox>
#include <QCloseEvent>
#include "Processing/vTools.h"

namespace Qylon{
class vToolsWidget : public QWidget
{
    Q_OBJECT
public:
    vToolsWidget(vTools *vTools=nullptr, QWidget *parent=nullptr);
    void loadRecipeConfiguration(Pylon::DataProcessing::CRecipe *recipe);

signals:
    void requestOutputControl();

public slots:
    void updateOutputControl();

private:
    vTools *vTools;
    vTools::ResultFilter outputList;
    QVBoxLayout *layout;
    QVBoxLayout *layoutOutputControl;
};
}

#endif // VTOOLSWIDGET_H
