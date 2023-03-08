HEADERS += \
    $$PWD/controller/pid/dynamic_clamp_pid.hpp \
    $$PWD/controller/pid/static_clamp_pid.hpp \
    $$PWD/controller/pid/inc_pid.hpp \
    
   
SOURCES += \
    $$PWD/controller/pid/dynamic_clamp_pid.cpp \
    $$PWD/controller/pid/static_clamp_pid.cpp \
    $$PWD/controller/pid/inc_pid.cpp \

HEADERS += \
    $$PWD/controller/adrc/ladrc_1st.hpp \
   
SOURCES += \
    $$PWD/controller/adrc/ladrc_1st.cpp \


HEADERS += \
    $$PWD/math/coordinate.h \
   
SOURCES += \
    $$PWD/math/coordinate.c \