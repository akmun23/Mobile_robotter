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
    QPainter painter;
    painter.begin(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Check if there is new data to read (status == 1)
    QSqlQuery query;
    if (query.exec("SELECT status FROM lidar_data WHERE id = 1") && query.next()) {
        int statusValue = query.value(0).toInt();

        if (statusValue == 1) {
            // There is new data, so retrieve it
            if (query.exec("SELECT angle, distance FROM lidar_data WHERE id != 1")) {
                while (query.next()) {
                    double angle = query.value(0).toDouble();
                    double distance = query.value(1).toDouble();
                    helper->lidarRead(angle, distance);
                    helper->paint(&painter, event, elapsed);
                }
            } else {
                qDebug() << "Error retrieving data:" << query.lastError().text();
            }

            // Mark data as read by setting status to 0
            if (!query.exec("UPDATE lidar_data SET status = 0 WHERE id = 1")) {
                qDebug() << "Error updating status flag:" << query.lastError().text();
            }

            // Clear the database except for id = 0 (this keeps id = 0 data)
            if (!query.exec("DELETE FROM lidar_data WHERE id != 1")) {
                qDebug() << "Error clearing data:" << query.lastError().text();
            }
        }
    } else {
        qDebug() << "Error checking data status:" << query.lastError().text();
    }

    // Paint robot or any additional visuals
    helper->paintRobot(&painter, event, elapsed);
    painter.end();
}
