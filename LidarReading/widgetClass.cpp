#include "widgetClass.h"
#include <QPainter>
#include <QTimer>
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlQuery>
#include <QtSql/QSqlError>
#include <QDebug>

Widget::Widget(Helper *helper, QWidget *parent) : QWidget(parent), helper(helper) {
    elapsed = 0;
    setFixedSize(getScrnSize());

    QSqlDatabase db = QSqlDatabase::addDatabase("QMYSQL");
    db.setHostName("localhost");       // Set the correct hostname
    db.setDatabaseName("lidar_db");    // Set your database name
    db.setUserName("aksel");           // Set the MySQL username
    db.setPassword("hua28rdc");        // Set the MySQL password

    if (!db.open()) {
        qDebug() << "Database error:" << db.lastError().text();
    }
}

void Widget::animate() {
    elapsed = (elapsed + qobject_cast<QTimer*>(sender())->interval()) % 1000;
    update();
}

void Widget::paintEvent(QPaintEvent *event) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    QSqlQuery query;
    if (query.exec("SELECT angle, distance FROM lidar_data WHERE id != 1")) {
        while (query.next()) {
            double angle = query.value(0).toDouble();
            double distance = query.value(1).toDouble();
            helper->lidarRead(angle, distance);
        }
        if (!query.exec("UPDATE lidar_data SET status = 0 WHERE id = 1")) {
            qDebug() << "Error updating status flag:" << query.lastError().text();
        }
        if (!query.exec("DELETE FROM lidar_data WHERE id != 1")) {
            qDebug() << "Error clearing data:" << query.lastError().text();
        }
    } else {
        qDebug() << "Error retrieving data:" << query.lastError().text();
    }

    helper->paint(&painter, event, elapsed);
    helper->paintRobot(&painter, event, elapsed);
}
