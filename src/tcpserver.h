#ifndef TCPSERVER_H
#define TCPSERVER_H

#include<qt5/QtNetwork/QTcpServer>
#include<qt5/QtNetwork/QTcpSocket>
#include<qt5/QtNetwork/QUdpSocket>
#include<qt5/QtCore/QObject>
#include<ros/ros.h>
#include <genquad/ESCControl.h>
#include <geometry_msgs/Vector3.h>
#include <genquad/bmp280_dat.h>
#include <genquad/mpu9250_dat.h>
#include <vector>

class TCPServer : public QObject
{
    Q_OBJECT

public:
    explicit TCPServer(ros::NodeHandle *noderef, QObject *parent = nullptr);
    ~TCPServer();

    quint16 msg_port = 5400;
    quint16 ctrl_port = 5401;


signals:
    void dataReceived(QByteArray);


private slots:
    void newConnectionMsg();
    void disconnectedMsg();
    void readyReadMsg();

    void newConnectionCtrl();
    void disconnectedCtrl();
    void readyReadCtrl();


private:
    QString msg_data_received;
    QString ctrl_data_received;
    //std::vector<int> ctrl_data_handler;
    int ctrl_data_handler[4];

    QTcpServer *server_msg;
    QTcpServer *server_ctrl;

    QTcpSocket *socket_msg;
    QTcpSocket *socket_ctrl;

    QHash<QString, int> reverse_hash_msg;
    QHash<QString, int> reverse_hash_ctrl;



    //ROS
    ros::NodeHandle node;

    ros::Subscriber rand_dat1_sub_handle;
    ros::Subscriber rand_dat2_sub_handle;
    ros::Subscriber pressure_sub_handle;
    ros::Subscriber imu_sub_handle;
    ros::Subscriber temp_sub_handle;
    ros::Subscriber mag_sub_handle;

    ros::ServiceClient esc_client_handle;

    //callbacks
    void update_data1(const geometry_msgs::Vector3::ConstPtr& data);
    void update_data2(const geometry_msgs::Vector3::ConstPtr& data);
    void update_bmp280(const genquad::bmp280_dat::ConstPtr& data);
    void update_mpu9250(const genquad::mpu9250_dat::ConstPtr& data);

    //dataHandler
    geometry_msgs::Vector3 data1_realtime;
    geometry_msgs::Vector3 data2_realtime;

    genquad::bmp280_dat bmp280_dat_realtime;
    genquad::mpu9250_dat mpu9250_dat_realtime;


    //bufferSpace
    geometry_msgs::Vector3 data1_buffer;
    geometry_msgs::Vector3 data2_buffer;

    genquad::bmp280_dat bmp280_dat_buffer;
    genquad::mpu9250_dat mpu9250_dat_buffer;



};

#endif // TCPSERVER_H
