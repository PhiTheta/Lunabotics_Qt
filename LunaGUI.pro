#-------------------------------------------------
#
# Project created by QtCreator 2012-11-08T13:46:05
#
#-------------------------------------------------

QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Lunabotics
TEMPLATE = app
ICON = bender.icns

SOURCES += main.cpp\
        mainwindow.cpp \
    preferencedialog.cpp \
    occupancygraphicsitem.cpp \
    laserscan.cpp \
    SleepSimulator.cpp

HEADERS  += mainwindow.h \
    preferencedialog.h \
    constants.h \
    occupancygraphicsitem.h \
    laserscan.h \
    SleepSimulator.h

FORMS    += mainwindow.ui \
    preferencedialog.ui

OTHER_FILES +=

RESOURCES += \
    LunaGUIResources.qrc
