include Common/CommonDefs.mak

.PHONY: all

NETPROJ = 
ifneq "$(realpath /usr/bin/gmcs)" ""
	NETPROJ += Boxes.net
endif

all: $(NETPROJ) TrackPad PointViewer Boxes Players SceneAnalysis SingleControl PointServer Boxes.java CircleControl 


.PHONY: TrackPad
TrackPad:
	$(MAKE) -C ../TrackPad

.PHONY: PointViewer
PointViewer:
	$(MAKE) -C ../PointViewer

.PHONY: Boxes
Boxes:
	$(MAKE) -C ../Boxes

.PHONY: Players
Players:
	$(MAKE) -C ../Players

.PHONY: SceneAnalysis
SceneAnalysis:
	$(MAKE) -C ../SceneAnalysis

.PHONY: SingleControl
SingleControl:
	$(MAKE) -C ../SingleControl

.PHONY: PointServer
PointServer:
	$(MAKE) -C ../PointServer

.PHONY: Boxes.java
Boxes.java:
	$(MAKE) -C ../Boxes.java

.PHONY: Boxes.net
Boxes.net:
	$(MAKE) -C ../Boxes.net

.PHONY: CircleControl
CircleControl:
	$(MAKE) -C ../CircleControl
