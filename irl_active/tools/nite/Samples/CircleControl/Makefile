OSTYPE := $(shell uname -s)

SRC_FILES = \
	main.cpp \
	kbhit.cpp \
	signal_catch.cpp

INC_DIRS += ../../../../../Samples/CircleControl

DEFINES += USE_GLUT

ifeq ("$(OSTYPE)","Darwin")
        LDFLAGS += -framework OpenGL -framework GLUT
else
        USED_LIBS += glut
endif

EXE_NAME = Sample-CircleControl
include ../NiteSampleMakefile

