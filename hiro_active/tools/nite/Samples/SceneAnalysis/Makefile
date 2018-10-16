OSTYPE := $(shell uname -s)

SRC_FILES = \
	main.cpp \
	SceneDrawer.cpp \
	kbhit.cpp \
	signal_catch.cpp

INC_DIRS += ../../../../../Samples/SceneAnalysis

EXE_NAME = Sample-SceneAnalysis

ifeq "$(GLES)" "1"
DEFINES += USE_GLES
USED_LIBS += GLES_CM IMGegl srv_um
SRC_FILES += opengles.cpp
else
DEFINES += USE_GLUT

ifeq ("$(OSTYPE)","Darwin")
        LDFLAGS += -framework OpenGL -framework GLUT
else
        USED_LIBS += glut
endif
endif

include ../NiteSampleMakefile

