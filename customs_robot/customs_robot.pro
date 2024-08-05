QT       += core gui

greaterThan(QT_MAJOR_VERSION, 5): QT += core5compat
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11
CONFIG += c++14

QMAKE_LFLAGS += -no-pie
QMAKE_CXXFLAGS += -mavx

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS


DEFINES += CODE_VERSION_MAIN
#DEFINES += CODE_VERSION_VISUAL

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
    LLog/LLog.cpp \
    common/common.cpp \
    common/image_check_box.cpp \
    customs_robot.cpp \
    main.cpp \
    dialog.cpp \
    page/main_page.cpp \
    param_cfg_page.cpp \
    robot_arm/component/axis_state.cpp \
    robot_arm/component/drive/array_lidar/tof_sense_m.cpp \
    robot_arm/component/drive/laser_ranging/laser_ranging_jrt.cpp \
    robot_arm/component/drive/laser_ranging/laser_ranging_jrt_bb2x.cpp \
    robot_arm/page/robot_arm_page.cpp \
    robot_arm/ranging_laser.cpp \
    robot_arm/ranging_laser_benewake.cpp \
    robot_arm/ranging_laser_long.cpp \
    robot_arm/robot_arm.cpp \
    robot_arm/zmcaux.cpp \
    robot_chassis/chassis_state_data.cpp \
    robot_chassis/ndc8_tcp_client.cpp \
    robot_chassis/nmc_tcp_client.cpp \
    robot_chassis/page/robot_chassis_page.cpp \
    robot_chassis/robot_chassis.cpp \
    serial_communication.cpp \
    task_management/page/task_management_page.cpp \
    task_management/task_management.cpp \
    tcp_client.cpp \
    test_demo/test_tcp_work.cpp \
    test_demo/test_tcp_work2.cpp \
    visual_identity/page/visual_identity_page.cpp \
    visual_identity/visual_identity.cpp \
    work_base.cpp

HEADERS += \
    LLog/LLog.h \
    common/common.h \
    common/image_check_box.h \
    customs_robot.h \
    dialog.h \
    page/main_page.h \
    param_cfg_page.h \
    robot_arm/component/axis_state.h \
    robot_arm/component/drive/array_lidar/tof_sense_m.h \
    robot_arm/component/drive/laser_ranging/laser_ranging_jrt.h \
    robot_arm/component/drive/laser_ranging/laser_ranging_jrt_bb2x.h \
    robot_arm/page/robot_arm_page.h \
    robot_arm/ranging_laser.h \
    robot_arm/ranging_laser_benewake.h \
    robot_arm/ranging_laser_long.h \
    robot_arm/robot_arm.h \
    robot_arm/zmcaux.h \
    robot_arm/zmotion.h \
    robot_chassis/chassis_state_data.h \
    robot_chassis/ndc8_tcp_client.h \
    robot_chassis/nmc_tcp_client.h \
    robot_chassis/page/robot_chassis_page.h \
    robot_chassis/robot_chassis.h \
    serial_communication.h \
    task_management/page/task_management_page.h \
    task_management/task_management.h \
    tcp_client.h \
    test_demo/test_tcp_work.h \
    test_demo/test_tcp_work2.h \
    visual_identity/page/visual_identity_page.h \
    visual_identity/visual_identity.h \
    work_base.h

contains(DEFINES, CODE_VERSION_VISUAL){
    SOURCES += visual_identity/objDetection.cpp
    HEADERS += visual_identity/objDetection.h
}

FORMS += \
    dialog.ui \
    page/main_page.ui \
    param_cfg_page.ui \
    robot_arm/page/robot_arm_page.ui \
    robot_chassis/page/robot_chassis_page.ui \
    task_management/page/task_management_page.ui \
    visual_identity/page/visual_identity_page.ui


INCLUDEPATH += /usr/local/include \
               /usr/local/include/src/ \
               /usr/include/libxml2 \
               /usr/local/include/opencv \
               /usr/local/include/opencv4 \
               /usr/include/eigen3

LIBS += -L/usr/local/share \
        -L/usr/local/bin \
        /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_core.so \
        /usr/local/lib/libopencv_imgproc.so \
        /usr/local/lib/libopencv_imgcodecs.so

LIBS += -L/usr/local/lib -lopencv_dnn -lopencv_ml -lopencv_objdetect \
     -lopencv_stitching -lopencv_calib3d -lopencv_features2d -lopencv_highgui \
     -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_imgproc -lopencv_flann \
     -lopencv_core -lrealsense2 -lxml2
#     -lydlidar_sdk

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

greaterThan(QT_MAJOR_VERSION,4){
        TARGET_ARCH=$${QT_ARCH}
}else{
        TARGET_ARCH=$${QMAKE_HOST.arch}
}
contains(TARGET_ARCH, x86_64){
    unix:!macx: LIBS += -L$$PWD/libzmotion/64/ -lzmotion

    INCLUDEPATH += $$PWD/libzmotion/64
    DEPENDPATH += $$PWD/libzmotion/64
}else{
    unix:!macx: LIBS += -L$$PWD/libzmotion/32/ -lzmotion

    INCLUDEPATH += $$PWD/libzmotion/32
    DEPENDPATH += $$PWD/libzmotion/32
}

#INCLUDEPATH += /usr/local/include \
#               /usr/local/include/opencv4
#LIBS += /usr/local/lib/libopencv_highgui.so \
#        /usr/local/lib/libopencv_core.so    \
#        /usr/local/lib/libopencv_imgproc.so \
#        /usr/local/lib/libopencv_imgcodecs.so
#LIBS += -L/usr/local/lib -lopencv_core -lopencv_imgcodecs -lopencv_highgui

RESOURCES += \
    png.qrc


