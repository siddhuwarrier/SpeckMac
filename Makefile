#
#  Makefile for SpeckMac
#
#  ** This file was automatically generated by the command:
#  opp_makemake -f -N -b /home/s0567031/work/Castalia -c /home/s0567031/work/Castalia/config/Castalia.config -I. -u Cmdenv -n -r -I/home/s0567031/work/Castalia/src/helpStructures -I/home/s0567031/work/Castalia/src/Node/Communication/Network -I/home/s0567031/work/Castalia/src/Node/Communication/MAC -I/home/s0567031/work/Castalia/src/Node/Communication/Radio -I/home/s0567031/work/Castalia/src/Node/Application -I/home/s0567031/work/Castalia/src/Wireless_Channel -I/home/s0567031/work/Castalia/src/Node/Resource_Manager
#

# Name of target to be created (-o option)
TARGET = SpeckMac

# User interface (uncomment one) (-u option)
USERIF_LIBS=$(CMDENV_LIBS)
# USERIF_LIBS=$(TKENV_LIBS)

# .ned or .h include paths with -I
INCLUDE_PATH= -I. -I/home/s0567031/work/Castalia/src/helpStructures -I/home/s0567031/work/Castalia/src/Node/Communication/Network -I/home/s0567031/work/Castalia/src/Node/Communication/MAC -I/home/s0567031/work/Castalia/src/Node/Communication/Radio -I/home/s0567031/work/Castalia/src/Node/Application -I/home/s0567031/work/Castalia/src/Wireless_Channel -I/home/s0567031/work/Castalia/src/Node/Resource_Manager

# misc additional object and library files to link
EXTRA_OBJS=

# object files from other directories to link with (wildcard needed to prevent "no such file *.o" errors)
EXT_DIR_OBJS=

# time stamps of other directories (used as dependency)
EXT_DIR_TSTAMPS= /home/s0567031/work/Castalia/src/helpStructures/.tstamp /home/s0567031/work/Castalia/src/Node/Communication/Network/.tstamp /home/s0567031/work/Castalia/src/Node/Communication/MAC/.tstamp /home/s0567031/work/Castalia/src/Node/Communication/Radio/.tstamp /home/s0567031/work/Castalia/src/Node/Application/.tstamp /home/s0567031/work/Castalia/src/Wireless_Channel/.tstamp /home/s0567031/work/Castalia/src/Node/Resource_Manager/.tstamp

# Additional libraries (-L option -l option)
LIBS=

#------------------------------------------------------------------------------
# Import generic settings from /home/s0567031/work/Castalia/config/Castalia.config
include /home/s0567031/work/Castalia/config/Castalia.config

MSGCOPTS= $(INCLUDE_PATH)

#------------------------------------------------------------------------------

# subdirectories to recurse into
SUBDIRS= 

# object files in this directory
OBJS=  SpeckMacModule.o

# header files generated (from msg files)
GENERATEDHEADERS= 

#------------------------------------------------------------------------------

$(TARGET): $(OBJS) subdirs Makefile .tstamp
	@# do nothing

.tstamp: $(OBJS)
	echo>.tstamp

$(OBJS) : $(GENERATEDHEADERS)


purify: $(OBJS) $(EXTRA_OBJS) $(EXT_DIR_TSTAMPS) subdirs Makefile
	purify $(CXX) $(LDFLAGS) $(OBJS) $(EXTRA_OBJS) $(EXT_DIR_OBJS) $(LIBS) -L$(OMNETPP_LIB_DIR) $(KERNEL_LIBS) $(USERIF_LIBS) $(SYS_LIBS_PURE) -o $(TARGET).pure

$(EXT_DIR_TSTAMPS):
	@echo
	@echo Error: $@ does not exist.
	@echo This means that at least the above dependency directory has not been built.
	@echo Maybe you need to do a top-level make?
	@echo
	@exit 1

.PHONY: subdirs $(SUBDIRS)

subdirs: $(SUBDIRS)

SpeckMacModule.o: SpeckMacModule.cc
	$(CXX) -c $(COPTS) SpeckMacModule.cc


#doc: neddoc doxy

#neddoc:
#	opp_neddoc -a

#doxy: doxy.cfg
#	doxygen doxy.cfg

generateheaders: $(GENERATEDHEADERS)
	for i in $(SUBDIRS); do (cd $$i && $(MAKE) generateheaders) || exit 1; done

clean:
	rm -f *.o *_n.cc *_n.h *_m.cc *_m.h .tstamp
	rm -f *.vec *.sca
	for i in $(SUBDIRS); do (cd $$i && $(MAKE) clean); done

depend:
	$(MAKEDEPEND) $(INCLUDE_PATH) -- *.cc
	# $(MAKEDEPEND) $(INCLUDE_PATH) -fMakefile.in -- *.cc
	for i in $(SUBDIRS); do (cd $$i && $(MAKE) depend) || exit 1; done

makefiles:
	# recreate Makefile
	opp_makemake -f  -N -b /home/s0567031/work/Castalia -c /home/s0567031/work/Castalia/config/Castalia.config -I. -u Cmdenv -n -r -I/home/s0567031/work/Castalia/src/helpStructures -I/home/s0567031/work/Castalia/src/Node/Communication/Network -I/home/s0567031/work/Castalia/src/Node/Communication/MAC -I/home/s0567031/work/Castalia/src/Node/Communication/Radio -I/home/s0567031/work/Castalia/src/Node/Application -I/home/s0567031/work/Castalia/src/Wireless_Channel -I/home/s0567031/work/Castalia/src/Node/Resource_Manager 
	for i in $(SUBDIRS); do (cd $$i && $(MAKE) makefiles) || exit 1; done

makefile-ins:
	# recreate Makefile.in
	opp_makemake -f -m  -N -b /home/s0567031/work/Castalia -c /home/s0567031/work/Castalia/config/Castalia.config -I. -u Cmdenv -n -r -I/home/s0567031/work/Castalia/src/helpStructures -I/home/s0567031/work/Castalia/src/Node/Communication/Network -I/home/s0567031/work/Castalia/src/Node/Communication/MAC -I/home/s0567031/work/Castalia/src/Node/Communication/Radio -I/home/s0567031/work/Castalia/src/Node/Application -I/home/s0567031/work/Castalia/src/Wireless_Channel -I/home/s0567031/work/Castalia/src/Node/Resource_Manager 
	for i in $(SUBDIRS); do (cd $$i && $(MAKE) makefile-ins) || exit 1; done

# "re-makemake" and "re-makemake-m" are deprecated, historic names of the above two targets
re-makemake: makefiles
re-makemake-m: makefile-ins


# DO NOT DELETE THIS LINE -- make depend depends on it.
SpeckMacModule.o: SpeckMacModule.cc \
  SpeckMacModule.h \
  /home/s0567031/work/Castalia/src/Node/Resource_Manager/ResourceGenericManager.h \
  /home/s0567031/work/Castalia/src/Node/Communication/Radio/RadioModule.h \
  /home/s0567031/work/Castalia/src/helpStructures/DebugInfoWriter.h
