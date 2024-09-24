#include "thread_3DMouse.h"
#include <QDebug>
#include <QMutexLocker>
#include <QEventLoop>
#include <QTimer>
#include <QProcess>
#include <iostream>
#include <spnav.h>

Thread3DMouse::Thread3DMouse(QObject *parent)
    : QThread(parent), quit(false)
{

}

Thread3DMouse::~Thread3DMouse(){
    quit=true;
    wait();
    qInfo() << "Thread 2 Destroyed";
}

void Thread3DMouse::startThread(){
    if (!isRunning())
        start();
}

void Thread3DMouse::run(){
    qDebug() << __PRETTY_FUNCTION__ << " start";
    if (spnav_open() == -1) {
        /* failed to connect to spacenavd */
        printf("failed to connect to spacenavd");
    } else {
        char buf[256];
        if (spnav_dev_name(buf, sizeof buf) != -1) {
            std::cout<<"Device: "<< buf<<std::endl;
        }
        spnav_event sev;
        while (true) {
            if (quit)
                break;
            // Using spnav_poll_event instead of spnav_wait_event to avoid blocking
            if (spnav_poll_event(&sev)) {
                //qInfo()<<spnav_fd();
                if (sev.type == SPNAV_EVENT_MOTION) {
                    emit motionEvent(sev.motion.x, sev.motion.y, sev.motion.z,
                                                         sev.motion.rx, sev.motion.ry, sev.motion.rz);
                } else { /* SPNAV_EVENT_BUTTON */
                    std::cout << "got button event: " << std::endl;
                }
            } else {
                QThread::msleep(10); // Sleep for a short period to avoid busy-waiting
            }
        }
        spnav_close();
        qInfo() << "loop end";
    }
}

void Thread3DMouse::close(){
    qDebug() << __PRETTY_FUNCTION__ << " " << __LINE__;
    quit=true;
    wait();
    quit=false;
}

