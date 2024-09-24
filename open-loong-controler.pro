#-------------------------------------------------
#
# Project created by QtCreator 2024-04-16T11:24:26
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = open-loong-controler
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
    global_var.cpp \
    k_ORL2/CoordinateTrans.cpp \
    k_ORL2/k_OLR2.cpp \
    k_ORL2/k_OLR2_initialize.cpp \
    k_ORL2/k_OLR2_terminate.cpp \
    k_ORL2/norm.cpp \
    k_ORL2/rtGetInf.cpp \
    k_ORL2/rtGetNaN.cpp \
    k_ORL2/rt_nonfinite.cpp \
        main.cpp \
        openloongcontroler.cpp \
    robotsystem.cpp \
    ArmFunction/udp_client.cpp \
    ArmFunction/udp_server.cpp \
    ArmFunction/RobotData.cpp \
    thread_demonstrator.cpp

HEADERS += \
    hand.h \
    k_ORL2/CoordinateTrans.h \
    k_ORL2/k_OLR2.h \
    k_ORL2/k_OLR2_initialize.h \
    k_ORL2/k_OLR2_terminate.h \
    k_ORL2/k_OLR2_types.h \
    k_ORL2/norm.h \
    k_ORL2/rtGetInf.h \
    k_ORL2/rtGetNaN.h \
    k_ORL2/rt_defines.h \
    k_ORL2/rt_nonfinite.h \
    k_ORL2/rtwtypes.h \
        openloongcontroler.h \
    robotsystem.h \
    ArmFunction/udp_client.h \
    ArmFunction/udp_server.h \
    ArmFunction/RobotData.h \
    thread_demonstrator.h

FORMS += \
    openloongcontroler.ui

DISTFILES += \
    my_qss.qss

RESOURCES += \
    pic.qrc

#LIBS += \
#    -lspnav
