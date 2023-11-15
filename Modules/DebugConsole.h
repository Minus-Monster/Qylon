#ifndef DEBUGCONSOLE_H
#define DEBUGCONSOLE_H
#include <QPlainTextEdit>
#include <QTextBlock>
#include <QTextCursor>
#include <QCoreApplication>

class DebugConsole : public QPlainTextEdit{
    Q_OBJECT
public:
    DebugConsole(){
        qRegisterMetaType<QTextBlock>("QTextBlock");
        qRegisterMetaType<QTextCursor>("QTextCursor");
        setWindowTitle("Debug Console");
    }
public slots:
    void appendText(const QString &text){
        this->appendPlainText(text);
        QCoreApplication::postEvent(this, new QEvent(QEvent::User));
    }
};

#endif // DEBUGCONSOLE_H
