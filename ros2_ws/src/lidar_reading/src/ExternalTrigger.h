#ifndef EXTERNALTRIGGER_H
#define EXTERNALTRIGGER_H

#include <QApplication>
#include <QtWidgets>
#include <QWidget>
#include <QWindow>
#include <QObject>

class ExternalTrigger : public QObject{
    Q_OBJECT

public:
    void emitTrigger(){
        emit triggerUpdate();
    }

public slots:

    void update();

    void updateTrigger(){
        update();
    }

signals:
    void triggerUpdate();
};



#endif // EXTERNALTRIGGER_H
