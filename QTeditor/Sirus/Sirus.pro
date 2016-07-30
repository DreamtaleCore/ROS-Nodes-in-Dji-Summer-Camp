QT += core
QT -= gui

CONFIG += c++11

TARGET = Sirus
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp

# README here first! ~~
# Assume all the libraries below you have setup
# Tips:
#   1. eigen3 is the base lib of aruco, you need setup it first
#   2. arduino is also can be eidted by qt to higlight and auto-compelete
#      If you wanna use the Serial in arduino, just define "HardwareSerial Serial"
#       globally
#   3. the onboard and guidance sdk include path may need be adjusted  as your wish

INCLUDEPATH += /opt/ros/indigo/include \        # the ros include headers and libs
                /usr/local/include \            # the opemcv && aruco include headers and libs
                /usr/local/include/eigen3 \     # eigen3
                ../../dji_sdk-master/include \              # dij onboard sdk
                ../../Guidance-SDK-ROS-master/include \     # dji guidance sdk
#               YOUR_INSTALL_PATH/arduino-1.6.9/hardware/arduino/avr/cores/arduino

LIBS += /usr/local/lib/*.so \       # the opencv && arudo softlink(.so)
        /opt/ros/indigo/lib/*.so    # the ros softlink(.so) nearly useless

