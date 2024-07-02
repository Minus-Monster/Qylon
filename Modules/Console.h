#ifndef CONSOLE_H
#define CONSOLE_H
#include <QTextEdit>
#include <QToolBar>
#include <QFontComboBox>
#include <QVBoxLayout>
#include <QColorDialog>
#include <QTextCursor>
#include <QTextBlock>
#include <QDebug>
#include <QMutex>
#include <QMutexLocker>

namespace Qylon{
class Console : public QWidget
{
    Q_OBJECT
public:
    Console(QWidget *parent =nullptr) : QWidget(parent), fontSize(12){
        textEdit = new QTextEdit(this);
        textEdit->setReadOnly(true);
        textEdit->setFrameShape(QFrame::Box);
        textEdit->setFontFamily("Arial");

        toolBar = new QToolBar("Console Tools", this);

        // Increase font size action
        increaseFontSizeAction = new QAction(QIcon(":/Resources/Icon/icons8-increase-font-48.png"),"", this);
        connect(increaseFontSizeAction, &QAction::triggered, this, &Console::increaseFontSize);
        toolBar->addAction(increaseFontSizeAction);

        // Decrease font size action
        decreaseFontSizeAction = new QAction(QIcon(":/Resources/Icon/icons8-decrease-font-48.png"),"", this);
        connect(decreaseFontSizeAction, &QAction::triggered, this, &Console::decreaseFontSize);
        toolBar->addAction(decreaseFontSizeAction);

        // Layout for the widget
        QVBoxLayout *layout = new QVBoxLayout;
        layout->addWidget(toolBar);
        layout->addWidget(textEdit);
        layout->setContentsMargins(0,0,0,0);

        setLayout(layout);
    }
    void append(QString string){
        QMutexLocker locker(&mutex);
        if(textEdit->document()->blockCount() > 300){
            QTextCursor cursor(textEdit->document());
            cursor.movePosition(QTextCursor::Start);
            cursor.movePosition(QTextCursor::Down, QTextCursor::MoveAnchor, 0);
            cursor.select(QTextCursor::BlockUnderCursor);
            cursor.removeSelectedText();
            cursor.deleteChar();
        }
        textEdit->append(string);
        QTextCursor cursor(textEdit->document());
        cursor.movePosition(QTextCursor::End);
        textEdit->setTextCursor(cursor);
    }

private slots:
    void increaseFontSize(){
        fontSize += 2;
        QFont font = textEdit->font();
        font.setPointSize(fontSize);
        textEdit->setFont(font);

    }
    void decreaseFontSize(){
        if (fontSize > 1) {
            fontSize -= 2;
            QFont font = textEdit->font();
            font.setPointSize(fontSize);
            textEdit->setFont(font);
        }
    }

private:
    int fontSize;
    QMutex mutex;
    QTextEdit *textEdit;
    QToolBar *toolBar;
    QAction *increaseFontSizeAction;
    QAction *decreaseFontSizeAction;
};
}

#endif // CONSOLE_H
