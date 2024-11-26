#ifndef VTOOLSWIDGET_H
#define VTOOLSWIDGET_H
#ifdef VTOOLS_ENABLED

#include <QWidget>
#include <QDebug>
#include <QVBoxLayout>
#include <QComboBox>
#include <QLabel>
#include <QGroupBox>
#include <QCheckBox>
#include <QCloseEvent>

#include "vTools/vTools.h"

namespace Qylon{
class vToolsWidget : public QWidget
{
    Q_OBJECT
public:
    vToolsWidget(vTools *parentVTools=nullptr, QWidget *parent=nullptr);
    void loadRecipeConfiguration(Pylon::DataProcessing::CRecipe *recipe);

signals:
    void requestOutputControl();

public slots:
    void updateOutputControl();

private:
    vTools::ResultFilter outputList;
    QVBoxLayout *layout;
    QVBoxLayout *layoutOutputControl;
};
}
#endif
#endif // VTOOLSWIDGET_H
