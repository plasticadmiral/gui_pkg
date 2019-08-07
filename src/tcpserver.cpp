#include <iostream>
#include "tcpserver.h"
#include <qt5/QtCore/QDataStream>
#include <qt5/QtCore/QBuffer>
#include <qt5/QtCore/QString>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <genquad/ESCControl.h>
#include <genquad/bmp280_dat.h>
#include <genquad/mpu9250_dat.h>
#include <string>

//#include <boost/bind.hpp>
class BlockWriter
{
public:
    BlockWriter(QIODevice *io)
    {
        buffer.open(QIODevice::WriteOnly);
        this->io = io;
        _stream.setVersion(QDataStream::Qt_4_8);
        _stream.setDevice(&buffer);
        _stream << quint64(0);
    }

    ~BlockWriter()
    {
        _stream.device()->seek(0);
        _stream << static_cast<quint64>(buffer.size());

        io->write(buffer.buffer());
    }

    QDataStream &stream()
    {
        return _stream;
    }

private:
    QBuffer buffer;
    QDataStream _stream;
    QIODevice *io;
};


class BlockReader
{
public:
    BlockReader(QIODevice *io)
    {
        buffer.open(QIODevice::ReadWrite);
        _stream.setVersion(QDataStream::Qt_4_8);
        _stream.setDevice(&buffer);

        qint64 blockSize;

        readMax(io, sizeof(blockSize));
        buffer.seek(0);
        _stream >> blockSize;

        readMax(io, blockSize);
        buffer.seek(sizeof(blockSize));
    }

    QDataStream& stream()
    {
        return _stream;
    }

private:
    void readMax(QIODevice *io, qint64 n)
    {
        while (buffer.size() < n) {
            buffer.write(io->read(n - buffer.size()));
        }
    }

    QBuffer buffer;
    QDataStream _stream;
};



TCPServer::TCPServer(ros::NodeHandle *noderef, QObject *parent) :
    QObject(parent), node(*noderef)
{

    rand_dat1_sub_handle = node.subscribe<geometry_msgs::Vector3>(
                "/random/data1", 100,
                &TCPServer::update_data1, this);
    //[](geometry_msgs::Vector3::ConstPtr& data){std::cout<<data->x<<std::endl});
    //[](const auto& data){std::cout<<data->x<<std::endl});


    rand_dat2_sub_handle = node.subscribe<geometry_msgs::Vector3>(
                "/random/data2", 100,
                &TCPServer::update_data2, this);
    //[](geometry_msgs::Vector3::ConstPtr& data){std::cout<<data->x<<std::endl});
    //[](const auto& data){std::cout<<data->x<<std::endl});

    pressure_sub_handle = node.subscribe<genquad::bmp280_dat>(
                "/Press_and_Temp", 100,
                &TCPServer::update_bmp280, this);

    imu_sub_handle = node.subscribe<genquad::mpu9250_dat>(
                "MPU9250_and_AK8963", 100,
                &TCPServer::update_mpu9250, this);

    esc_client_handle = node.serviceClient<genquad::ESCControl>(
                "/manipulateESC");


    server_msg = new QTcpServer(this);
    server_ctrl = new QTcpServer(this);

    std::cout << "Listening for Message Requests:" << server_msg->listen(QHostAddress::Any, msg_port) <<std::endl;
    std::cout << "Listening for Control Requests:" << server_ctrl->listen(QHostAddress::Any, ctrl_port) <<std::endl;;


    connect(server_msg, SIGNAL(newConnection()), this, SLOT(newConnectionMsg()));
    connect(server_ctrl, SIGNAL(newConnection()), this, SLOT(newConnectionCtrl()));

    //ros::init(argc, argv, "GUI_node");
    reverse_hash_msg.insert("data1x", 1);
    reverse_hash_msg.insert("data1y", 2);
    reverse_hash_msg.insert("data1z", 3);

    reverse_hash_msg.insert("data2x", 4);
    reverse_hash_msg.insert("data2y", 5);
    reverse_hash_msg.insert("data2z", 6);

    reverse_hash_msg.insert("baro", 7);

    reverse_hash_msg.insert("bmptemp", 8);

    reverse_hash_msg.insert("accelx", 9);
    reverse_hash_msg.insert("accely", 10);
    reverse_hash_msg.insert("accelz", 11);

    reverse_hash_msg.insert("mputemp", 12);

    reverse_hash_msg.insert("gyrox", 13);
    reverse_hash_msg.insert("gyroy", 14);
    reverse_hash_msg.insert("gyroz", 15);

    reverse_hash_msg.insert("magx", 16);
    reverse_hash_msg.insert("magy", 17);
    reverse_hash_msg.insert("magz", 18);

    //    reverse_hash_ctrl.insert("trtlup", 1);
    //    reverse_hash_ctrl.insert("trtldown", 2);
    //    reverse_hash_ctrl.insert("ptchfrnt", 3);
    //    reverse_hash_ctrl.insert("ptchbck", 4);
    //    reverse_hash_ctrl.insert("rlllft", 5);
    //    reverse_hash_ctrl.insert("rllrht", 6);
    //    reverse_hash_ctrl.insert("ywlft", 7);
    //    reverse_hash_ctrl.insert("ywrht", 8);

    //ros::spin();


}

TCPServer::~TCPServer()
{
    server_msg->deleteLater();
    server_ctrl->deleteLater();
}

void TCPServer::newConnectionMsg()
{
    while (server_msg->hasPendingConnections())
    {
        std::cout<<"incoming connection for messages!" <<std::endl;
        socket_msg = server_msg->nextPendingConnection();
        connect(socket_msg, SIGNAL(readyRead()), this, SLOT(readyReadMsg()));
        connect(socket_msg, SIGNAL(disconnected()), this, SLOT(disconnectedMsg()));
    }
}

void TCPServer::disconnectedMsg()
{
    std::cout << "disconnected messages port!" <<std::endl;
    disconnect(socket_msg, SIGNAL(readyRead()), this, SLOT(readyReadMsg()));
    disconnect(socket_msg, SIGNAL(disconnected()), this, SLOT(disconnectedMsg()));
    socket_msg->deleteLater();

}

void TCPServer::readyReadMsg()
{
    std::cout << "Read!" <<std::endl;
    BlockReader(socket_msg).stream() >> msg_data_received;
    std::cout <<"received data request: " << msg_data_received.toStdString() <<std::endl;

    switch(reverse_hash_msg.value(msg_data_received))
    {
    case 1:
        std::cout << "responding go hash1 (data1x) request!"<<std::endl;
        ros::spinOnce();
        std::cout <<"response is: "<< data1_buffer.x<<std::endl;;
        BlockWriter(socket_msg).stream() << QString::number(data1_buffer.x);
        break;

    case 2:
        std::cout << "responding go hash2 (data1y) request!" <<std::endl;
        std::cout <<"response is: "<< data1_buffer.y<<std::endl;;
        BlockWriter(socket_msg).stream() << QString::number(data1_buffer.y);
        break;

    case 3:
        std::cout << "responding go hash3 (data1z) request!" <<std::endl;
        std::cout <<"response is: "<< data1_buffer.z<<std::endl;;
        BlockWriter(socket_msg).stream() << QString::number(data1_buffer.z);
        break;

    case 4:
        std::cout << "responding go hash4 (data2x) request!" <<std::endl;
        ros::spinOnce();
        std::cout <<"response is: "<< data2_buffer.x<<std::endl;;
        BlockWriter(socket_msg).stream() << QString::number(data2_buffer.x);
        break;

    case 5:
        std::cout << "responding go hash5 (data2y) request!" <<std::endl;
        std::cout <<"response is: "<< data2_buffer.y<<std::endl;;
        BlockWriter(socket_msg).stream() << QString::number(data2_buffer.y);
        break;

    case 6:
        std::cout << "responding go hash6 (data2z) request!" <<std::endl;
        std::cout <<"response is: "<< data2_buffer.z<<std::endl;;
        BlockWriter(socket_msg).stream() << QString::number(data2_buffer.z);
        break;

    case 7:
        std::cout << "responding to hash7 (baro) request!" <<std::endl;
        ros::spinOnce();
        BlockWriter(socket_msg).stream() << QString::number(bmp280_dat_buffer.press.fluid_pressure);
        break;

    case 8:
        std::cout << "responding to hash8 (bmptemp) request!" <<std::endl;
        std::cout<<"yoooo hereeee!!! "<<bmp280_dat_buffer.temp.temperature;
        BlockWriter(socket_msg).stream() << QString::number(bmp280_dat_buffer.temp.temperature);
        break;
    case 9:
        std::cout << "responding to hash9 (accelx) request!" <<std::endl;
        ros::spinOnce();
        BlockWriter(socket_msg).stream() << QString::number(mpu9250_dat_buffer.imu.linear_acceleration.x);
        break;

    case 10:
        std::cout << "responding to hash10 (accely) request!" <<std::endl;
        BlockWriter(socket_msg).stream() << QString::number(mpu9250_dat_buffer.imu.linear_acceleration.y);
        break;

    case 11:
        std::cout << "responding to hash11 (accelz) request!" <<std::endl;
        BlockWriter(socket_msg).stream() << QString::number(mpu9250_dat_buffer.imu.linear_acceleration.z);
        break;

    case 12:
        std::cout << "responding to hash12 (mputemp) request!" <<std::endl;
        BlockWriter(socket_msg).stream() << QString::number(mpu9250_dat_buffer.temp.temperature);
        break;

    case 13:
        std::cout << "responding to hash13 (gyrox) request!" <<std::endl;
        BlockWriter(socket_msg).stream() << QString::number(mpu9250_dat_buffer.imu.angular_velocity.x);
        break;

    case 14:
        std::cout << "responding to hash14 (gyroy) request!" <<std::endl;
        BlockWriter(socket_msg).stream() << QString::number(mpu9250_dat_buffer.imu.angular_velocity.y);
        break;

    case 15:
        std::cout << "responding to hash15 (gyroz) request!" <<std::endl;
        BlockWriter(socket_msg).stream() << QString::number(mpu9250_dat_buffer.imu.angular_velocity.z);
        break;

    case 16:
        std::cout << "responding to hash16 (magx) request!" <<std::endl;
        BlockWriter(socket_msg).stream() << QString::number(mpu9250_dat_buffer.mag.magnetic_field.x);
        break;

    case 17:
        std::cout << "responding to hash17 (magy) request!" <<std::endl;
        BlockWriter(socket_msg).stream() << QString::number(mpu9250_dat_buffer.mag.magnetic_field.y);
        break;

    case 18:
        std::cout << "responding to hash18 (magz) request!" <<std::endl;
        BlockWriter(socket_msg).stream() << QString::number(mpu9250_dat_buffer.mag.magnetic_field.z);
        break;


    default:
        std::cout << "unrecognized command received.. Please check sequence!" <<std::endl;

    }

    //BlockWriter(socket_msg).stream()<<data;
    socket_msg->flush();

}

void TCPServer::newConnectionCtrl()
{
    while (server_ctrl->hasPendingConnections())
    {
        std::cout <<"incoming connection for control!" <<std::endl;
        socket_ctrl = server_ctrl->nextPendingConnection();
        connect(socket_ctrl, SIGNAL(readyRead()), this, SLOT(readyReadCtrl()));
        connect(socket_ctrl, SIGNAL(disconnected()), this, SLOT(disconnectedCtrl()));
    }

}



void TCPServer::disconnectedCtrl()
{

    std::cout << "disconnected control port!" <<std::endl;
    disconnect(socket_ctrl,SIGNAL(readyRead()), this, SLOT(readyReadCtrl()));
    disconnect(socket_ctrl,SIGNAL(disconnected()), this, SLOT(disconnectedCtrl()));
    socket_ctrl->deleteLater();
}


void TCPServer::readyReadCtrl()
{
    BlockReader(socket_ctrl).stream() >> ctrl_data_received;
    std::cout <<"received control request: " << ctrl_data_received.toStdString() <<std::endl;
    int count = 0;
    std::string buffer = "";
    genquad::ESCControl esc_handle;
    //received_ctrl_data = "1000 200 -50 700";
    for (auto x : ctrl_data_received.toStdString())
    {
        if(x == ' ')
        {


            ctrl_data_handler[count] = stoi(buffer);
            //ctrl_data_handler.push_back(stoi(buffer));
            count++;
            buffer = "";
        }
        else
        {
            buffer = buffer + x;
            std::cout<<buffer <<std::endl;
        }
    }
    ctrl_data_handler[count] = stoi(buffer);
    std::cout<<"ctrl data handler op is: "<<ctrl_data_handler[0]<<" "<<ctrl_data_handler[1]<<" "<<ctrl_data_handler[2]<<" "<<ctrl_data_handler[3]<<std::endl;
    esc_handle.request.thrustPercentage = ctrl_data_handler[0];
    esc_handle.request.rollPercentage = ctrl_data_handler[1];
    esc_handle.request.pitchPercentage = ctrl_data_handler[2];
    esc_handle.request.yawPercentage = ctrl_data_handler[3];

    std::cout<<"request ip is: "<<esc_handle.request.thrustPercentage<<" "<<esc_handle.request.rollPercentage<<" "<<esc_handle.request.pitchPercentage<<" "<<esc_handle.request.yawPercentage<<std::endl;

    if(esc_client_handle.call(esc_handle))
    {
        std::string str = std::to_string(esc_handle.response.frontLeftPW) + ' ' +
                std::to_string(esc_handle.response.frontRightPW) + ' ' +
                std::to_string(esc_handle.response.backLeftPW) + ' ' +
                std::to_string(esc_handle.response.backRightPW);

        BlockWriter(socket_ctrl).stream() << QString::fromStdString(str);
    }
    else
    {
        std::cout << "unable to handle call from client for ESC! Check sequence!" <<std::endl;
    }

    //    ui->ESC_FL_label->setNum(pwmSignals[0]);
    //    ui->ESC_FR_label->setNum(pwmSignals[1]);
    //    ui->ESC_FL_label->setNum(pwmSignals[2]);
    //    ui->ESC_BR_label->setNum(pwmSignals[3]);
    //    switch (reverse_hash_ctrl.value(ctrl_data_received))
    //    {
    //    case 1:
    //        BlockWriter(socket_ctrl).stream() << QString::number(data2_buffer.z);
    //        break;

    //    case 2:

    //        break;

    //    case 3:

    //        break;

    //    case 4:

    //        break;

    //    case 5:

    //        break;

    //    case 6:

    //        break;

    //    default:
    //        qDebug() << "unrecognized command received.. Please check sequence!";
    //        break;

    //    }
}

void TCPServer::update_data1(const geometry_msgs::Vector3::ConstPtr& data)
{
    //std::cout << "update_data1 is called" <<std::endl;
    data1_buffer.x = data->x;
    data1_buffer.y = data->y;
    data1_buffer.z = data->z;

}

void TCPServer::update_data2(const geometry_msgs::Vector3::ConstPtr& data)
{
    //std::cout << "update_data2 is called" <<std::endl;
    data2_buffer.x = data->x;
    data2_buffer.y = data->y;
    data2_buffer.z = data->z;


}

void TCPServer::update_bmp280(const genquad::bmp280_dat::ConstPtr& data)
{
    //std::cout <<"update_bmp280 is called" <<std::endl;
    bmp280_dat_buffer.header = data->header;
    bmp280_dat_buffer.temp = data->temp;
    bmp280_dat_buffer.press = data->press;

}

void TCPServer::update_mpu9250(const genquad::mpu9250_dat::ConstPtr& data)
{
    //std::cout <<"udpdate_mpu9250 is called" <<std::endl;
    mpu9250_dat_buffer.header = data->header;
    mpu9250_dat_buffer.temp = data->temp;
    mpu9250_dat_buffer.imu = data->imu;
    mpu9250_dat_buffer.mag = data->mag;
}




