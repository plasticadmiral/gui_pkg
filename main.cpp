#include <qt5/QtCore/QCoreApplication>
#include "src/tcpserver.h"
#include <ros/ros.h>

int main(int argc, char* argv[])
{
    QCoreApplication a(argc, argv);
    ros::init(argc, argv, "TCPServerNode");
    ros::NodeHandle node;
    TCPServer socket(&node);

    return a.exec();
}

