#-------------------------------------------------
#
# Project created by QtCreator 2014-05-22T15:06:04
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = EigenNonLinearOptTests
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    SolvePnPFunctor.cpp

HEADERS += \
    GenericFunctor.h \
    SolvePnPFunctor.h


INCLUDEPATH += /home/dushy/eigen-3.2.1/
