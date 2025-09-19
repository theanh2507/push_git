#include <QApplication>
#include "mainwindow.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);
    MainWindow window(&app);
    window.show();

    int ret = app.exec();

    // Shutdown ROS2 sau Qt
    rclcpp::shutdown();
    return ret;
}
