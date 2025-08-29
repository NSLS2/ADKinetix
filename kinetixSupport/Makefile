TOP=..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# Pull in the necessary libraries and headers from your PVCam SDK installation.

SRC_DIRS += $(PVCAM_INCLUDE)

INC += pvcam.h master.h

LIB_INSTALLS_Linux += $(PVCAM_LIB)/libpvcam.so
LIB_INSTALLS_Linux += $(PVCAM_LIB)/libpvcamDDI.so

LIB_INSTALLS_WIN32 += $(PVCAM_LIB)/pvcam64.lib


#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

