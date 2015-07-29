#-------------------------------------------------
#
# Project created by QtCreator 2015-07-20T21:21:52
#
#-------------------------------------------------

QT       += core gui
QT       += network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PoseEstimation
TEMPLATE = app

SOURCES += main.cpp\
        mainwindow.cpp \
    mylibrary.cpp \
    theiap3pkneip.cpp \
    opengv.cpp \
    ../../../opengv-master/src/absolute_pose/CentralAbsoluteAdapter.cpp

HEADERS  += mainwindow.h \
    mylibrary.h \
    theiap3pkneip.h \
    opengv.h

FORMS    += mainwindow.ui

win32: LIBS += -L$$PWD/../../Libraries/opengv-master/lib/ -lopengv

INCLUDEPATH += $$PWD/../../Libraries/opengv-master/include
DEPENDPATH += $$PWD/../../Libraries/opengv-master/include

INCLUDEPATH += $$PWD/../../Libraries/opengv-master/third_party
DEPENDPATH += $$PWD/../../Libraries/opengv-master/third_party

win32:!win32-g++: PRE_TARGETDEPS += $$PWD/../../Libraries/opengv-master/lib/opengv.lib
else:win32-g++: PRE_TARGETDEPS += $$PWD/../../Libraries/opengv-master/lib/libopengv.a
