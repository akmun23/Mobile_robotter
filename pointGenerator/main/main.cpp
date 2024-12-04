#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlQuery>
#include <QtSql/QSqlError>
#include <QDebug>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <random>

void insertData(QSqlDatabase& db, double distance, double angle, double robot_x, double robot_y, double robot_yaw) {
    QSqlQuery query(db);

    // Forbered SQL-forespørgslen
    query.prepare("INSERT INTO lidar_data (distance, angle, robot_x, robot_y, robot_yaw) "
                  "VALUES (?, ?, ?, ?, ?)");

    // Bind værdierne
    query.addBindValue(distance);
    query.addBindValue(angle);
    query.addBindValue(robot_x);
    query.addBindValue(robot_y);
    query.addBindValue(robot_yaw);

    // Udfør indsættelsen
    if (!query.exec()) {
        qDebug() << "Fejl ved indsættelse af data:" << query.lastError().text();
    } else {
        qDebug() << "Data indsat korrekt!";
    }
}

double generateRandomFloat(double min, double max) {
    // Opretter en random engine baseret på systemets tid som seed
    static std::default_random_engine engine(static_cast<unsigned int>(std::time(nullptr)));

    // Opretter en uniform distribution af flydende tal mellem min og max
    std::uniform_real_distribution<double> distribution(min, max);

    // Returnerer et tilfældigt flydende tal mellem min og max
    return distribution(engine);
}

int main() {
    // Opret forbindelse til MySQL-databasen via Qt
    QSqlDatabase db = QSqlDatabase::addDatabase("QMYSQL");
    db.setHostName("localhost");
    db.setDatabaseName("lidar_db");  // Erstat med dit databasenavn
    db.setUserName("aksel");        // Erstat med dit brugernavn
    db.setPassword("hua28rdc");    // Erstat med din adgangskode

    if (!db.open()) {
        qDebug() << "Fejl ved oprettelse af forbindelse:" << db.lastError().text();
        return 1;
    }

    QSqlQuery query;
    query.prepare("DELETE FROM lidar_data");
    if (!query.exec()) {
        qDebug() << "Fejl ved slet af data:" << query.lastError().text();
    }
        else {
    }

    // Initialiser tilfældig talgenerator
    srand(time(0));

    // Generer og indsæt 10 tilfældige punkter i databasen

    std::vector<std::vector<double>> wallFrags;

    for (int i = 0; i < 200; ++i) {
        double distance = 1.5;
        double angle = M_PI/2;
        double robot_x = 1.5 - (i * 3.0/200);
        double robot_y = 0;
        double robot_yaw = 0;

        // Indsæt de genererede værdier i databasen
        insertData(db, 0, 0, robot_x, robot_y, 0);
        insertData(db, distance, angle, robot_x, robot_y, robot_yaw);

        // Tilføj punktet til wallFrags
        wallFrags.push_back({distance, angle, robot_x, robot_y, robot_yaw});

        for (size_t j = 0; j < wallFrags.size(); ++j) {
            // Convert the wall fragment polar coordinates (distance, angle) to Cartesian (x_global, y_global)
            double x_global = wallFrags[j][0] * cos(wallFrags[j][1]) + wallFrags[j][2];
            double y_global = wallFrags[j][0] * sin(wallFrags[j][1]) + wallFrags[j][3];

            // Calculate the relative position of the wall fragment to the robot's new position
            double x_relative = x_global - robot_x;
            double y_relative = y_global - robot_y;

            // Calculate the new distance and angle from the robot's position to the wall fragment
            double new_distance = sqrt(x_relative * x_relative + y_relative * y_relative);
            double new_angle = atan2(y_relative, x_relative);

            // Insert the transformed data into the database
            insertData(db, new_distance, new_angle, robot_x, robot_y, robot_yaw);
        }

    }

    // Luk forbindelsen
    db.close();

    return 0;
}
