/**
 * ADKinetix.h
 *
 * ADKinetix EPICS areaDetector driver header file
 *
 * Author: Jakub Wlodek
 *
 * Created: 10-Mar-2024
 * Last Updated: 26-Jul-2024
 * Copyright (c): Brookhaven National Laboratory 2024
 */

// Include guard
#ifndef ADKINETIX_H
#define ADKINETIX_H

// Driver version numbers
#define ADKINETIX_VERSION 1
#define ADKINETIX_REVISION 3
#define ADKINETIX_MODIFICATION 0

// Standard Includes
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <condition_variable>
#include <mutex>

// EPICS includes
#include <cantProceed.h>
#include <epicsEvent.h>
#include <epicsExit.h>
#include <epicsExport.h>
#include <epicsMutex.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <iocsh.h>

#include "ADDriver.h"

// PvCam includes
#include "master.h"
#include "pvcam.h"

// Helper data structure definitions
#include "ADKinetixDS.h"

// Size of the continuous acquisition circular buffer
#define KTX_CIRC_BUFF_SIZE 50

// Driver name
static const char* driverName = "ADKinetix";

// Temperature and Fan signals
#define KTX_TemperatureString "KTX_TEMP"
#define KTX_FanSpeedString "KTX_FAN_SPEED"

// Acquisition new frame timeout signals
#define KTX_StopAcqOnTimeoutString "KTX_STOP_ACQ_ON_TO"
#define KTX_WaitForFrameTimeoutString "KTX_WAIT_FOR_FRAME_TO"

// Communication interface signal
#define KTX_CommInterfaceString "KTX_INTERFACE"

// Readout mode signals
#define KTX_MinExpResString "KTX_MIN_EXP_RES"
#define KTX_ReadoutModeString "KTX_READOUT_MODE"
#define KTX_ApplyReadoutModeString "KTX_APPLY_MODE"
#define KTX_ModeValidString "KTX_MODE_VALID"
#define KTX_ReadoutPortIdxString "KTX_READOUT_PORT_IDX"
#define KTX_ReadoutPortDescString "KTX_READOUT_PORT_DESC"
#define KTX_SpeedIdxString "KTX_SPEED_IDX"
#define KTX_SpeedDescString "KTX_SPEED_DESC"
#define KTX_GainIdxString "KTX_GAIN_IDX"
#define KTX_GainDescString "KTX_GAIN_DESC"

// Three supported trigger modes
typedef enum {
    KTX_TRIG_INTERNAL = 0,
    KTX_TRIG_EDGE = 1,
    KTX_TRIG_GATE = 2,
} KTX_TRIG_MODE;

// Supported interface modes
typedef enum {
    KTX_INTF_UNKNOWN = 0,
    KTX_INTF_USB = 1,
    KTX_INTF_USB_1_1 = 2,
    KTX_INTF_USB_2_0 = 3,
    KTX_INTF_USB_3_0 = 4,
    KTX_INTF_USB_3_1 = 5,
    KTX_INTF_PCIE = 6,
    KTX_INTF_PCIE_x1 = 7,
    KTX_INTF_PCIE_x4 = 8,
    KTX_INTF_PCIE_x8 = 9,
    KTX_INTF_VIRTUAL = 10,
    KTX_INTF_ETHERNET = 11,
} KTX_COMM_INTF;

// Potential minimum exposure resolution
typedef enum {
    KTX_MIN_EXP_RES_S = 0,
    KTX_MIN_EXP_RES_MS = 1,
    KTX_MIN_EXP_RES_US = 2,
} KTX_MIN_EXP_RES;

class ADKinetix : public ADDriver {
   public:
    ADKinetix(int deviceIndex, const char* portName);

    /* These are the methods that we override from ADDriver */
    virtual asynStatus readEnum(asynUser* pasynUser, char* strings[], int values[],
                                int severities[], size_t nElements, size_t* nIn);
    virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser* pasynUser, epicsFloat64 value);
    void report(FILE* fp, int details);
    void monitorThread();
    void acquisitionThread();
    ~ADKinetix();
    bool waitForEofEvent(uns32 timeoutMs);

   protected:
    int KTX_Temperature;
#define FIRST_KTX_PARAM KTX_Temperature
    int KTX_FanSpeed;
    int KTX_StopAcqOnTimeout;
    int KTX_WaitForFrameTimeout;
    int KTX_CommInterface;
    int KTX_MinExpRes;
    int KTX_ReadoutMode;
    int KTX_ApplyReadoutMode;
    int KTX_ModeValid;
    int KTX_ReadoutPortIdx;
    int KTX_ReadoutPortDesc;
    int KTX_SpeedIdx;
    int KTX_SpeedDesc;
    int KTX_GainIdx;
    int KTX_GainDesc;
#define LAST_KTX_PARAM KTX_GainDesc

   private:
    void reportKinetixError(const char* functionName);

    bool isParamAvailable(int16 hcam, uns32 paramID, const char* paramName);
    bool readEnumeration(int16 hcam, NVPC* pNvpc, uns32 paramID, const char* paramName);
    void printSpeedTable();
    bool getSpeedTable();
    void updateImageFormat();
    void updateReadoutPortDesc();
    void selectSpeedTableMode();

    void acquireStart();
    void acquireStop();
    void updateCameraRegion();
    void getCurrentFrameDimensions(size_t* dims);
    asynStatus setMinExpRes(KTX_MIN_EXP_RES currentExpRes, KTX_MIN_EXP_RES minExpRes);

    NDDataType_t getCurrentNDBitDepth();
    CameraContext* cameraContext;
    bool acquisitionActive = false;
    bool speedTableModeSelected = false;
    bool pollingTemperature = false;
    bool monitoringActive = false;
    int deviceIndex;
    void* frameBuffer = nullptr;
    epicsThreadId monitorThreadId, acquisitionThreadId;
};

#define NUM_KTX_PARAMS ((int) (&LAST_KTX_PARAM - &FIRST_KTX_PARAM + 1))

#endif
