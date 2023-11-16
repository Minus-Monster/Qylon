#ifndef DEBUGCONSOLE_H
#define DEBUGCONSOLE_H
#include "qdebug.h"
#include <QPlainTextEdit>
#include <QTextBlock>
#include <QTextCursor>
#include <QCoreApplication>

namespace Qylon{
class DebugConsole : public QTextEdit{
    Q_OBJECT
public:
    DebugConsole(){
        qRegisterMetaType<QTextBlock>("QTextBlock");
        qRegisterMetaType<QTextCursor>("QTextCursor");
        setWindowTitle("Debug Console");
        setReadOnly(true);
    }
public slots:
    virtual void append(QString message){
//        qDebug() << message << sender();
        QTextEdit::append(message);
    }
};
}
#endif // DEBUGCONSOLE_H
