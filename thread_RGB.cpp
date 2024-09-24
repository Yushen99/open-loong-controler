#include "thread_RGB.h"


ThreadRGB::ThreadRGB(QObject *parent)
    : QThread(parent),quit(false)
{

}

ThreadRGB::~ThreadRGB(){
    mutex.lock();
    quit=true;
    mutex.unlock();
    wait();
    qInfo()<<"Thread 3 Destoryed";
}

void ThreadRGB::startThread(){
    if(!isRunning())
        start();
}

void ThreadRGB::run(){
    qDebug()<<__PRETTY_FUNCTION__<<"() start";
    boost::asio::io_service io_service;
    boost::asio::serial_port serial_port(io_service, "/dev/ttyUSB0");
    serial_port.set_option(boost::asio::serial_port::baud_rate(9600));
    serial_port.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    serial_port.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    serial_port.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    serial_port.set_option(boost::asio::serial_port::character_size(8));
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    std::thread sendThread([this, &serial_port](){
        bool flag=true;
        while (!quit) {
            if(flag){
                sendData(serial_port);
            }else{
                unsigned char sendData[] = { 0xAA, 0x0, 0x0, 0x0, 0xAA };
                boost::system::error_code error;
                boost::asio::write(serial_port, boost::asio::buffer(sendData, sizeof(sendData)), error);
            }
            std::cout<<"data sent"<<std::endl;
            flag=!flag;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        unsigned char sendData[] = { 0xAA, 0x0, 0x0, 0x0, 0xAA };
        boost::system::error_code error;
        boost::asio::write(serial_port, boost::asio::buffer(sendData, sizeof(sendData)), error);

    });

//    std::thread receiveThread([this, &serial_port](){
//        while (!quit) {
//            receiveData(serial_port);
//        }
//    });

    sendThread.join();
//    receiveThread.join();
}

void ThreadRGB::sendData(boost::asio::serial_port& serial_port) {
    unsigned char data[] = { 0xAA, R_value, G_value, B_value, 0x0 };
    data[4] = 0xAA + R_value + G_value + B_value;
    boost::system::error_code error;
    boost::asio::write(serial_port, boost::asio::buffer(data, sizeof(data)), error);
    if (error) {
        std::cout << "Send Error: " << error.message() << std::endl;
    }
}

void ThreadRGB::receiveData(boost::asio::serial_port& serial_port) {
    char buf[100];
    boost::system::error_code error;
    //size_t len = serial_port.read_some(boost::asio::buffer(buf), error);
    if (error) {
        std::cout << "Receive Error: " << error.message() << std::endl;
    } else {
        std::cout<<"data received: "<<static_cast<int>(buf[0])<<std::endl;
    }
}



void ThreadRGB::close(){
    qDebug()<<__PRETTY_FUNCTION__<<"()"<<__LINE__;
    quit = true;
    wait();
    quit = false;
}
