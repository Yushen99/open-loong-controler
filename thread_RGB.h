#ifndef THREAD_RGB_H
#define THREAD_RGB_H
#include <QThread>
#include <QMutex>
#include <QString>
#include <QDebug>
#include <QMutex>
#include <QEventLoop>
#include <QTimer>
#include <QProcess>
#include <boost/asio.hpp>
#include <cfloat>
#include <cstdio>
#include <cstdlib>
#include <iostream>

extern std::atomic<uint8_t> R_value;
extern std::atomic<uint8_t> G_value;
extern std::atomic<uint8_t> B_value;

class ThreadRGB : public QThread
{
    Q_OBJECT

public:
    explicit ThreadRGB(QObject *parent = nullptr);
    ~ThreadRGB() override;

    void  startThread();
    void run() Q_DECL_OVERRIDE;
    void close();

signals:
    void error(const QString &s);
    void timeout(const QString &s);

private:
    QMutex mutex;
    bool quit;
    QMutex mtx_value;

    void sendData(boost::asio::serial_port& serial_port);

    void receiveData(boost::asio::serial_port& serial_port);

    unsigned char calculateChecksum(const unsigned char* data, size_t length);
public:


signals:


};

#endif // THREAD_RGB_H
