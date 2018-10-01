OSTYPE := $(shell uname -s)

SRC_FILES = \
	main.cpp \
	PointDrawer.cpp \
	signal_catch.cpp

INC_DIRS += ../../../../../Samples/PointViewer

EXE_NAME = Sample-PointViewer

ifeq "$(GLES)" "1"
DEFINES += USE_GLES
USED_LIBS += GLES_CM IMGegl srv_um
SRC_FILES += \
	kbhit.cpp \
	opengles.cpp
else
DEFINES += USE_GLUT

ifeq ("$(OSTYPE)","Darwin")
        LDFLAGS += -framework OpenGL -framework GLUT
else
        USED_LIBS += glut
endif
endif
include ../NiteSampleMakefile

