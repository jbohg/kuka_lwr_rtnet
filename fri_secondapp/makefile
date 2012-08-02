#
#  (C) KUKA Roboter Gmbh, Augsburg, Germany 2010
#
#   (gmake) compatible Makefile for building 
#           "KUKA.FastResearchInterface" secondApp
#    Take it as it is... 
#  NOTE: If your system does not support "socklen_t" 
#   (calling parameter of recvfrom), remove -DHAVE_SOCKLEN_T

all: friSecondApp


SECONDAPPSOURCES=fri_secondapp.cpp friremote.cpp friudp.cpp

FRIREMOTEDIR=../fri_remote
FRIOPTS = -DHAVE_SOCKLEN_T

CXXFLAGS = -I$(FRIREMOTEDIR) $(FRIOPTS)
vpath % $(FRIREMOTEDIR)

friSecondApp: $(SECONDAPPSOURCES)
	$(CXX)  -o $@ $(CXXFLAGS) $?

clean:
	rm friSecondApp



