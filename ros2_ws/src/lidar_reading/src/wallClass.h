#ifndef WALLCLASS_H
#define WALLCLASS_H

#include <QApplication>
#include <QtWidgets>
#include <QWindow>
#include <qbrush.h>

enum rectType{ empty, background, wall };

class Wall : public QRect{
    using QRect::QRect;

public:

    enum squareType{
        typeBackground,
        typeWall,
        typeEmpty
    };

    Wall(QPoint corner_, QSize size_, int orientation) : QRect(corner_, size_){
        type = typeBackground;
    }

    void setType(squareType newType){
        type = newType;
    }

    squareType getType(){
        return type;
    }

private:
    squareType type;
    QPoint corner;
    QSize size;
    rectType rt;

};

#endif // WALLCLASS_H
