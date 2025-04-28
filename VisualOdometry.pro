QT = core gui

CONFIG += c++17 cmdline

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        SCV4LFILE.cpp \
        calibrationcamera.cpp \
        main.cpp \
        stereovisualodometry.cpp

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

INCLUDEPATH += "D:/OpenCV/opencv/build/include"
DEPENDPATH += "D:/OpenCV/opencv/build/include"

LIBS += D:/OpenCV/OpenCV_bin/bin/libopencv_core4100.dll
LIBS += D:/OpenCV/OpenCV_bin/bin/libopencv_highgui4100.dll
LIBS += D:/OpenCV/OpenCV_bin/bin/libopencv_imgcodecs4100.dll
LIBS += D:/OpenCV/OpenCV_bin/bin/libopencv_imgproc4100.dll
LIBS += D:/OpenCV/OpenCV_bin/bin/libopencv_features2d4100.dll
LIBS += D:/OpenCV/OpenCV_bin/bin/libopencv_calib3d4100.dll
LIBS += D:/OpenCV/OpenCV_bin/bin/libopencv_videoio4100.dll
LIBS += D:/OpenCV/OpenCV_bin/bin/libopencv_video4100.dll

HEADERS += \
    SCV4LFILE.h \
    calibrationcamera.h \
    stereovisualodometry.h
