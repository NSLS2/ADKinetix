TOP=..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The files in this directory are installed first because they are
# used by PVCam driver

INC += pvcam.h master.h

LIB_INSTALLS_Linux += ../os/linux-x86_64/libpvcam.so
LIB_INSTALLS_Linux += ../os/linux-x86_64/libpvcam.so.2
LIB_INSTALLS_Linux += ../os/linux-x86_64/libpvcam.so.2.6

LIB_INSTALLS_Linux += ../os/linux-x86_64/libpvcamDDI.so
LIB_INSTALLS_Linux += ../os/linux-x86_64/libpvcamDDI.so.3
LIB_INSTALLS_Linux += ../os/linux-x86_64/libpvcamDDI.so.3.1

LIB_INSTALLS_WIN32 += ../os/windows-x64/pvcam64.lib


#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

