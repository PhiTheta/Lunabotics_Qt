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
    SleepSimulator.cpp \
    Telecommand.pb.cc \
    Twist.pb.cc \
    Telemetry.pb.cc \
    SteeringModeType.pb.cc \
    Point.pb.cc

HEADERS  += mainwindow.h \
    preferencedialog.h \
    constants.h \
    occupancygraphicsitem.h \
    laserscan.h \
    SleepSimulator.h \
    Telecommand.pb.h \
    Twist.pb.h \
    Telemetry.pb.h \
    SteeringModeType.pb.h \
    Point.pb.h

FORMS    += mainwindow.ui \
    preferencedialog.ui

RESOURCES += \
    LunaGUIResources.qrc

# Protobuf integration
#PROTOS = Telecommand.proto Telemetry.proto

#protobuf_header.name = protobuf header
#protobuf_header.input = PROTOS
#protobuf_header.output  = ${QMAKE_FILE_BASE}.pb.h
#protobuf_header.commands = protoc --cpp_out="." -I`dirname ${QMAKE_FILE_NAME}` ${QMAKE_FILE_NAME}
#protobuf_header.variable_out = GENERATED_FILES
#QMAKE_EXTRA_COMPILERS += protobuf_header

#protobuf_src.name  = protobuf src
#protobuf_src.input = PROTOS
#protobuf_src.output  = ${QMAKE_FILE_BASE}.pb.cc
#protobuf_src.depends  = ${QMAKE_FILE_BASE}.pb.h
#protobuf_src.commands = true
#protobuf_src.variable_out = GENERATED_SOURCES
#QMAKE_EXTRA_COMPILERS += protobuf_src

LIBS += -lprotobuf
