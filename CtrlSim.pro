QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

CONFIG += c++11
DEFINES += QCUSTOMPLOT_USE_OPENGL


# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \


HEADERS += \


FORMS += \


# include(page/page.pri)
include(module/module.pri)
include(example/DR_CAN/dr_can.pri)
include(page/page.pri)


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


win32:LIBS += -lOpengl32\
-lglu32
unix:LIBS += -lglut -lGLU



#Firstly,Set TARGET_ARCH variable.
greaterThan(QT_MAJOR_VERSION, 4) {
    TARGET_ARCH=$${QT_ARCH}
} else {
    TARGET_ARCH=$${QMAKE_HOST.arch}
}
#Secondly, use TARGET_ARCH to check.
contains(TARGET_ARCH, x86_64) {
    ARCHITECTURE = x64
    message("64-bit")
    LIBS += -L$$PWD/module/wave/lib/x64/ -lplot_widget
} else {
    ARCHITECTURE = x86
    message("32-bit")
    LIBS += -L$$PWD/module/wave/lib/x32/ -lplot_widget
}

INCLUDEPATH += $$PWD/module/wave/lib/inc
DEPENDPATH += $$PWD/module/wave/lib/inc
