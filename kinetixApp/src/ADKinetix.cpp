/**
 * ADKinetix.cpp
 *
 * Main source file for the ADKinetix EPICS areaDetector driver.
 *
 * Author: Jakub Wlodek
 *
 * Created: 10-Mar-2024
 * Last Updated: 07-Oct-2024
 * Copyright (c): Brookhaven National Laboratory 2024
 */

#include "ADKinetix.h"
/* PvCam includes */
#include "master.h"
#include "pvcam.h"

// Error message formatters
#define ERR(msg) \
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s: %s\n", driverName, functionName, msg)

#define ERR_ARGS(fmt, ...)                                                                    \
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s: " fmt "\n", driverName, functionName, \
              __VA_ARGS__);

// Warning message formatters
#define WARN(msg) \
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s::%s: %s\n", driverName, functionName, msg)

#define WARN_ARGS(fmt, ...)                                                                     \
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s::%s: " fmt "\n", driverName, functionName, \
              __VA_ARGS__);

// Info message formatters
#define INFO(msg) \
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DEVICE, "%s::%s: %s\n", driverName, functionName, msg)

#define INFO_ARGS(fmt, ...)                                                                      \
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DEVICE, "%s::%s: " fmt "\n", driverName, functionName, \
              __VA_ARGS__);

// Debug message formatters
#define DEBUG(msg) \
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s: %s\n", driverName, functionName, msg)

#define DEBUG_ARGS(fmt, ...)                                                                 \
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s: " fmt "\n", driverName, functionName, \
              __VA_ARGS__);

/**
 * @brief Function that instatiates a driver object. Called from IOC shell
 *
 */
extern "C" int ADKinetixConfig(int deviceIndex, const char *portName) {
    new ADKinetix(deviceIndex, portName);
    return (asynSuccess);
}

/**
 * @brief Wrapper C function passed to epicsThreadCreate to create temp monitor thread
 *
 * @param drvPvt Pointer to instance of ADKinetix driver object
 */
static void monitorThreadC(void *drvPvt) {
    ADKinetix *pPvt = (ADKinetix *) drvPvt;
    pPvt->monitorThread();
}

/**
 * @brief Wrapper C function passed to epicsThreadCreate to create acquisition thread
 *
 * @param drvPvt Pointer to instance of ADKinetix driver object
 */
static void acquisitionThreadC(void *drvPvt) {
    ADKinetix *pPvt = (ADKinetix *) drvPvt;
    pPvt->acquisitionThread();
}

/**
 * @brief Callback function called when exit is ran from IOC shell, deletes driver object instance
 *
 * @param drvPvt Pointer to instance of ADKinetix driver object
 */
static void exitCallbackC(void *drvPvt) {
    ADKinetix *pPvt = (ADKinetix *) drvPvt;
    delete pPvt;
}

/**
 * @brief Function that blocks until either timeout is reached, or new frame callback is detected
 *
 * @param timeoutMs Time to wait before returning in ms
 * @param pendingFrames Set to the number of frames still pending after this call
 * @param frameNr Set to the camera FrameNr of the consumed EOF event
 * @return true if frame successfully was awaited
 * @return false if acquisition was aborted or timed out
 */
bool ADKinetix::waitForEofEvent(uns32 timeoutMs, uns32 *pendingFrames, int32 *frameNr) {
    const char *functionName = "waitForEofEvent";

    std::unique_lock<std::mutex> lock(this->cameraContext->eofEvent.mutex);
    CameraContext *ctx =
        this->cameraContext;  // For lambda we need temporary local reference to ctx pointer
    this->cameraContext->eofEvent.cond.wait_for(
        lock, std::chrono::milliseconds(timeoutMs),
        [ctx]() { return !ctx->pendingFrameNrs.empty() || ctx->threadAbortFlag; });
    *pendingFrames = (uns32) this->cameraContext->pendingFrameNrs.size();
    if (!this->acquisitionActive) {
        DEBUG_ARGS("Acquisition completed on camera %d\n", this->cameraContext->hcam);
        return false;
    }
    if (this->cameraContext->pendingFrameNrs.empty()) {
        WARN_ARGS("Camera %d timed out waiting for a frame\n", this->cameraContext->hcam);
        return false;
    }
    *frameNr = this->cameraContext->pendingFrameNrs.front();
    this->cameraContext->pendingFrameNrs.pop_front();
    *pendingFrames = (uns32) this->cameraContext->pendingFrameNrs.size();
    return true;
}

/**
 * @brief Function that checks if camera supports SDK parameter
 *
 * @param hcam device ID
 * @param paramID parameter ID
 * @param paramName parameter name
 * @return true if parameter is supported, false otherwise
 */
bool ADKinetix::isParamAvailable(int16 hcam, uns32 paramID, const char *paramName) {
    if (!paramName) {
        return false;
    }

    rs_bool isAvailable;
    if (PV_OK != pl_get_param(hcam, paramID, ATTR_AVAIL, (void *) &isAvailable)) {
        printf("Error reading ATTR_AVAIL of %s\n", paramName);
        return false;
    }
    if (isAvailable == FALSE) {
        printf("Parameter %s is not available\n", paramName);
        return false;
    }

    return true;
}

/**
 * @brief Function that extracts enum names from PVCam parameter
 *
 * @param hcam deviceID
 * @param pNvpc pointer to NVPC table
 * @param paramID parameter ID
 * @param paramName parameterName
 * @return true if the enum table is not empty, false otherwise
 */
bool ADKinetix::readEnumeration(int16 hcam, NVPC *pNvpc, uns32 paramID, const char *paramName) {
    const char *functionName = "readEnumeration";
    if (!pNvpc || !paramName) return false;

    if (!isParamAvailable(hcam, paramID, paramName)) return false;

    uns32 count;
    if (PV_OK != pl_get_param(hcam, paramID, ATTR_COUNT, (void *) &count)) {
        const std::string msg = "pl_get_param(" + std::string(paramName) + ")";
        reportKinetixError(functionName);
        return false;
    }

    NVPC nvpc;
    for (uns32 i = 0; i < count; ++i) {
        // Retrieve the enum string length
        uns32 strLength;
        if (PV_OK != pl_enum_str_length(hcam, paramID, i, &strLength)) {
            const std::string msg = "pl_enum_str_length(" + std::string(paramName) + ")";
            reportKinetixError(functionName);
            return false;
        }

        // Allocate the destination string
        char *name = new (std::nothrow) char[strLength];
        if (!name) {
            ERR_ARGS("Unable to allocate memory for %s enum item name\n", paramName);
            return false;
        }

        // Get the string and value
        int32 value;
        if (PV_OK != pl_get_enum_param(hcam, paramID, i, &value, name, strLength)) {
            const std::string msg = "pl_get_enum_param(" + std::string(paramName) + ")";
            reportKinetixError(functionName);
            delete[] name;
            return false;
        }

        NVP nvp;
        nvp.value = value;
        nvp.name = name;
        nvpc.push_back(nvp);

        delete[] name;
    }
    pNvpc->swap(nvpc);

    return !pNvpc->empty();
}

/**
 * @brief Function that switches speed table mode based on PV selection.
 */
void ADKinetix::selectSpeedTableMode() {
    const char *functionName = "selectSpeedTableMode";

    // Get current speed table selection
    int readoutPortIdx, speedIdx, gainIdx;
    getIntegerParam(KTX_ReadoutPortIdx, &readoutPortIdx);
    getIntegerParam(KTX_SpeedIdx, &speedIdx);
    getIntegerParam(KTX_GainIdx, &gainIdx);

    // Update selection description
    updateReadoutPortDesc();

    // Configure speed table mode
    if (PV_OK != pl_set_param(this->cameraContext->hcam, PARAM_READOUT_PORT,
                              (void *) &this->cameraContext->speedTable[readoutPortIdx].value)) {
        reportKinetixError(functionName);
        this->speedTableModeSelected = false;
        return;
    }
    INFO_ARGS("Readout port set to '%s'",
              this->cameraContext->speedTable[readoutPortIdx].name.c_str());

    if (PV_OK !=
        pl_set_param(
            this->cameraContext->hcam, PARAM_SPDTAB_INDEX,
            (void *) &this->cameraContext->speedTable[readoutPortIdx].speeds[speedIdx].index)) {
        reportKinetixError(functionName);
        this->speedTableModeSelected = false;
        return;
    }
    INFO_ARGS("Readout speed set to %d ns per pix",
              this->cameraContext->speedTable[readoutPortIdx].speeds[speedIdx].pixTimeNs);

    if (PV_OK != pl_set_param(this->cameraContext->hcam, PARAM_GAIN_INDEX,
                              (void *) &this->cameraContext->speedTable[readoutPortIdx]
                                  .speeds[speedIdx]
                                  .gains[gainIdx]
                                  .index)) {
        reportKinetixError(functionName);
        this->speedTableModeSelected = false;
        return;
    }
    INFO_ARGS("Gain set to %s", this->cameraContext->speedTable[readoutPortIdx]
                                    .speeds[speedIdx]
                                    .gains[gainIdx]
                                    .name.c_str());

    char readoutModeStr[256];
    snprintf(
        readoutModeStr, 256, "Mode: %s @ %d nspp w/ %d bpp",
        this->cameraContext->speedTable[readoutPortIdx].name.c_str(),
        this->cameraContext->speedTable[readoutPortIdx].speeds[speedIdx].pixTimeNs,
        this->cameraContext->speedTable[readoutPortIdx].speeds[speedIdx].gains[gainIdx].bitDepth);
    setStringParam(KTX_ReadoutMode, readoutModeStr);

    // Determine image format based on mode
    if (this->cameraContext->speedTable[readoutPortIdx].speeds[speedIdx].gains[gainIdx].bitDepth !=
        8) {
        this->cameraContext->imageFormat = PL_IMAGE_FORMAT_MONO16;
    } else
        this->cameraContext->imageFormat = PL_IMAGE_FORMAT_MONO8;

    NDDataType_t dataType =
        this->cameraContext->imageFormat == PL_IMAGE_FORMAT_MONO8 ? NDUInt8 : NDUInt16;
    setIntegerParam(NDDataType, dataType);

    this->speedTableModeSelected = true;
    callParamCallbacks();
}

/**
 * @brief Utility function for printing speed table
 */
void ADKinetix::printSpeedTable() {
    const char *functionName = "printSpeedTable";
    if (this->cameraContext->speedTable.empty()) {
        ERR("No speed table read from detector!");
        return;
    }
    // Speed table has been created, print it out
    printf("  Speed table:\n");
    for (const auto &port : this->cameraContext->speedTable) {
        printf("  - port '%s', value %d\n", port.name.c_str(), port.value);
        for (const auto &speed : port.speeds) {
            printf("    - speed index %d, running at %f MHz\n", speed.index,
                   1000.0 / (float) speed.pixTimeNs);
            for (const auto &gain : speed.gains) {
                printf("      - gain index %d, %sbit-depth %d bpp\n", gain.index,
                       (gain.name.empty()) ? "" : ("'" + gain.name + "', ").c_str(), gain.bitDepth);
            }
        }
    }
    printf("\n");
}

/**
 * @brief Function that fetches supported speed table from camera
 *
 * @return true if table read correctly, false otherwise
 */
bool ADKinetix::getSpeedTable() {
    const char *functionName = "getSpeedTable";
    std::vector<SpdtabPort> table;

    NVPC ports;
    if (!readEnumeration(this->cameraContext->hcam, &ports, PARAM_READOUT_PORT,
                         "PARAM_READOUT_PORT"))
        return false;

    if (!isParamAvailable(this->cameraContext->hcam, PARAM_SPDTAB_INDEX, "PARAM_SPDTAB_INDEX"))
        return false;
    if (!isParamAvailable(this->cameraContext->hcam, PARAM_PIX_TIME, "PARAM_PIX_TIME"))
        return false;
    if (!isParamAvailable(this->cameraContext->hcam, PARAM_GAIN_INDEX, "PARAM_GAIN_INDEX"))
        return false;
    if (!isParamAvailable(this->cameraContext->hcam, PARAM_BIT_DEPTH, "PARAM_BIT_DEPTH"))
        return false;

    rs_bool isGainNameAvailable;
    if (PV_OK != pl_get_param(this->cameraContext->hcam, PARAM_GAIN_NAME, ATTR_AVAIL,
                              (void *) &isGainNameAvailable)) {
        ERR("Error reading ATTR_AVAIL of PARAM_GAIN_NAME\n");
        return false;
    }
    const bool isGainNameSupported = isGainNameAvailable != FALSE;

    // Iterate through available ports and their speeds
    for (size_t pi = 0; pi < ports.size(); pi++) {
        // Set readout port
        if (PV_OK != pl_set_param(this->cameraContext->hcam, PARAM_READOUT_PORT,
                                  (void *) &ports[pi].value)) {
            reportKinetixError(functionName);
            return false;
        }

        // Get number of available speeds for this port
        uns32 speedCount;
        if (PV_OK != pl_get_param(this->cameraContext->hcam, PARAM_SPDTAB_INDEX, ATTR_COUNT,
                                  (void *) &speedCount)) {
            reportKinetixError(functionName);
            return false;
        }

        SpdtabPort port;
        port.value = ports[pi].value;
        port.name = ports[pi].name;

        // Iterate through all the speeds
        for (int16 si = 0; si < (int16) speedCount; si++) {
            // Set camera to new speed index
            if (PV_OK !=
                pl_set_param(this->cameraContext->hcam, PARAM_SPDTAB_INDEX, (void *) &si)) {
                reportKinetixError(functionName);
                return false;
            }

            // Get pixel time (readout time of one pixel in nanoseconds) for the
            // current port/speed pair. This can be used to calculate readout
            // frequency of the port/speed pair.
            uns16 pixTime;
            if (PV_OK != pl_get_param(this->cameraContext->hcam, PARAM_PIX_TIME, ATTR_CURRENT,
                                      (void *) &pixTime)) {
                reportKinetixError(functionName);
                return false;
            }

            uns32 gainCount;
            if (PV_OK != pl_get_param(this->cameraContext->hcam, PARAM_GAIN_INDEX, ATTR_COUNT,
                                      (void *) &gainCount)) {
                reportKinetixError(functionName);
                return false;
            }

            SpdtabSpeed speed;
            speed.index = si;
            speed.pixTimeNs = pixTime;

            // Iterate through all the gains, notice it starts at value 1!
            for (int16 gi = 1; gi <= (int16) gainCount; gi++) {
                // Set camera to new gain index
                if (PV_OK !=
                    pl_set_param(this->cameraContext->hcam, PARAM_GAIN_INDEX, (void *) &gi)) {
                    reportKinetixError(functionName);
                    return false;
                }

                // Get bit depth for the current gain
                int16 bitDepth;
                if (PV_OK != pl_get_param(this->cameraContext->hcam, PARAM_BIT_DEPTH, ATTR_CURRENT,
                                          (void *) &bitDepth)) {
                    reportKinetixError(functionName);
                    return false;
                }

                SpdtabGain gain;
                gain.index = gi;
                gain.bitDepth = bitDepth;

                if (isGainNameSupported) {
                    char gainName[MAX_GAIN_NAME_LEN];
                    if (PV_OK != pl_get_param(this->cameraContext->hcam, PARAM_GAIN_NAME,
                                              ATTR_CURRENT, (void *) gainName)) {
                        reportKinetixError(functionName);
                        return false;
                    }

                    gain.name = gainName;
                }

                speed.gains.push_back(gain);
            }

            port.speeds.push_back(speed);
        }

        table.push_back(port);
    }

    this->cameraContext->speedTable.swap(table);
    return true;
}

/**
 * @brief Updates image format based on selected speed table mode
 */
void ADKinetix::updateImageFormat() {
    const char *functionName = "updateImageFormat";
    this->cameraContext->imageFormat = PL_IMAGE_FORMAT_MONO16;

    rs_bool isAvailable;
    if (PV_OK != pl_get_param(this->cameraContext->hcam, PARAM_IMAGE_FORMAT, ATTR_AVAIL,
                              (void *) &isAvailable))
        return;
    if (isAvailable == FALSE) return;

    int32 imageFormat;
    if (PV_OK != pl_get_param(this->cameraContext->hcam, PARAM_IMAGE_FORMAT, ATTR_CURRENT,
                              (void *) &imageFormat))
        return;

    DEBUG_ARGS("Detected image format %d for selected mode", imageFormat);
    this->cameraContext->imageFormat = imageFormat;
}

/**
 * @brief Callback function fired on each new frame event
 */
static void PV_DECL newFrameCallback(FRAME_INFO *pFrameInfo, CameraContext *ctx) {
    if (ctx == nullptr) return;

    // Queue every EOF so back-to-back frames are not coalesced into one event
    {
        std::lock_guard<std::mutex> lock(ctx->eofEvent.mutex);
        ctx->pendingFrameNrs.push_back(pFrameInfo != nullptr ? pFrameInfo->FrameNr : -1);
    }

    // Notify acquisition thread of new frame event
    ctx->eofEvent.cond.notify_all();
}

/**
 * @brief Updates the currently selected readout region for the camera
 */
void ADKinetix::updateCameraRegion() {
    const char *functionName = "updateCameraRegion";

    // Get region dimensions and binning from EPICS
    int regionStartX, regionStartY, regionSizeX, regionSizeY, binX, binY;

    getIntegerParam(ADMinX, &regionStartX);
    getIntegerParam(ADSizeX, &regionSizeX);
    getIntegerParam(ADMinY, &regionStartY);
    getIntegerParam(ADSizeY, &regionSizeY);
    getIntegerParam(ADBinX, &binX);
    getIntegerParam(ADBinY, &binY);

    int regionStopX = regionStartX + regionSizeX;
    int regionStopY = regionStartY + regionSizeY;

    // Update dimensions in context struct
    this->cameraContext->region.s1 = (uns16) regionStartX;
    this->cameraContext->region.s2 = (uns16) regionStopX - 1;
    this->cameraContext->region.sbin = (uns16) binX;
    this->cameraContext->region.p1 = (uns16) regionStartY;
    this->cameraContext->region.p2 = (uns16) regionStopY - 1;
    this->cameraContext->region.pbin = (uns16) binY;

    INFO_ARGS("Configured Region: (%d, %d) to (%d, %d) | Binning: (%d, %d)",
              (int) this->cameraContext->region.s1, (int) this->cameraContext->region.p1,
              (int) this->cameraContext->region.s2, (int) this->cameraContext->region.p2,
              (int) this->cameraContext->region.sbin, (int) this->cameraContext->region.pbin)
}

/**
 * @brief Functions for getting dimensions of current frame from context
 *
 * @param dims Pointer to array of size_t representing dimensions of each axis
 */
void ADKinetix::getCurrentFrameDimensions(size_t *dims) {
    dims[0] = (size_t) ((this->cameraContext->region.s2 - this->cameraContext->region.s1 + 1) /
                        this->cameraContext->region.sbin);
    dims[1] = (size_t) ((this->cameraContext->region.p2 - this->cameraContext->region.p1 + 1) /
                        this->cameraContext->region.pbin);
}

/**
 * @brief Main constructor for ADKinetix driver
 *
 * @param deviceIndex Device index as seen by PVCam
 * @param portName Unique asyn port name
 */
ADKinetix::ADKinetix(int deviceIndex, const char *portName)
    : ADDriver(portName, 1, NUM_KTX_PARAMS, 0, 0, asynEnumMask, asynEnumMask, ASYN_CANBLOCK, 1, 0,
               0) {
    const char *functionName = "ADKinetix";

    // Initialize new parameters in parameter library
    createParam(KTX_TemperatureString, asynParamFloat64, &KTX_Temperature);
    createParam(KTX_FanSpeedString, asynParamInt32, &KTX_FanSpeed);
    createParam(KTX_CommInterfaceString, asynParamInt32, &KTX_CommInterface);
    createParam(KTX_StopAcqOnTimeoutString, asynParamInt32, &KTX_StopAcqOnTimeout);
    createParam(KTX_WaitForFrameTimeoutString, asynParamInt32, &KTX_WaitForFrameTimeout);
    createParam(KTX_MinExpResString, asynParamInt32, &KTX_MinExpRes);
    createParam(KTX_ReadoutModeString, asynParamOctet, &KTX_ReadoutMode);
    createParam(KTX_ApplyReadoutModeString, asynParamInt32, &KTX_ApplyReadoutMode);
    createParam(KTX_ModeValidString, asynParamInt32, &KTX_ModeValid);
    createParam(KTX_ReadoutPortDescString, asynParamOctet, &KTX_ReadoutPortDesc);
    createParam(KTX_ReadoutPortIdxString, asynParamInt32, &KTX_ReadoutPortIdx);
    createParam(KTX_SpeedDescString, asynParamOctet, &KTX_SpeedDesc);
    createParam(KTX_SpeedIdxString, asynParamInt32, &KTX_SpeedIdx);
    createParam(KTX_GainIdxString, asynParamInt32, &KTX_GainIdx);
    createParam(KTX_GainDescString, asynParamOctet, &KTX_GainDesc);
    createParam(KTX_DroppedFramesString, asynParamInt32, &KTX_DroppedFrames);
    createParam(KTX_PendingFramesString, asynParamInt32, &KTX_PendingFrames);
    createParam(KTX_UniqueIdSourceString, asynParamInt32, &KTX_UniqueIdSource);
    createParam(KTX_CircBuffSizeString, asynParamInt32, &KTX_CircBuffSize);

    // Initialize the PVCam SDK
    if (!pl_pvcam_init()) {
        reportKinetixError(functionName);
        exit(1);
    }

    // Get current SDK version
    uns16 sdkVersion;
    pl_pvcam_get_ver(&sdkVersion);
    char sdkVersionStr[40], driverVersionStr[40], fwVersionStr[40], modelStr[40], vendorStr[40];
    snprintf(sdkVersionStr, 40, "%d.%d.%d", (sdkVersion >> 8 & 0xFF), (sdkVersion >> 4 & 0xF),
             (sdkVersion >> 0 & 0xF));
    setStringParam(ADSDKVersion, sdkVersionStr);

    // Track current version of the driver
    snprintf(driverVersionStr, 40, "%d.%d.%d", ADKINETIX_VERSION, ADKINETIX_REVISION,
             ADKINETIX_MODIFICATION);
    setStringParam(NDDriverVersion, driverVersionStr);

    // Identify number of connected cameras
    int16 numCameras = 0;
    if (!pl_cam_get_total(&numCameras)) {
        reportKinetixError(functionName);
    } else if (numCameras <= 0) {
        ERR("No cameras detected!");
    } else if (numCameras < deviceIndex) {
        ERR_ARGS("There are only %d detected cameras! Cannot open camera with index %d!",
                 numCameras, deviceIndex);
    } else {
        // Initialize fresh camera context
        this->cameraContext = new (std::nothrow) CameraContext();
        if (!this->cameraContext) {
            ERR("Failed to create camera context!");
        } else {
            // Open Camera
            pl_cam_get_name(deviceIndex, this->cameraContext->camName);
            if (!pl_cam_open(this->cameraContext->camName, &this->cameraContext->hcam,
                             OPEN_EXCLUSIVE)) {
                ERR_ARGS("Failed to open camera with name %s", this->cameraContext->camName);
            } else {
                this->cameraContext->isCamOpen = true;
                INFO_ARGS("Opened camera %s...", this->cameraContext->camName);

                // Camera is opened, collect model number, serial, and firmware information
                uns16 fwVersion;
                pl_get_param(this->cameraContext->hcam, PARAM_CAM_FW_VERSION, ATTR_CURRENT,
                             (void *) &fwVersion);
                snprintf(fwVersionStr, 40, "%d.%d", (fwVersion >> 8) & 0xFF,
                         (fwVersion >> 0) & 0xFF);
                setStringParam(ADFirmwareVersion, fwVersionStr);

                char serialNumber[MAX_ALPHA_SER_NUM_LEN];
                pl_get_param(this->cameraContext->hcam, PARAM_HEAD_SER_NUM_ALPHA, ATTR_CURRENT,
                             serialNumber);
                setStringParam(ADSerialNumber, serialNumber);

                pl_get_param(this->cameraContext->hcam, PARAM_VENDOR_NAME, ATTR_CURRENT,
                             (void *) vendorStr);
                setStringParam(ADManufacturer, vendorStr);

                pl_get_param(this->cameraContext->hcam, PARAM_PRODUCT_NAME, ATTR_CURRENT,
                             (void *) modelStr);
                setStringParam(ADModel, modelStr);

                // Attempt to switch to microsecond min exposure resolution
                setMinExpRes(KTX_MIN_EXP_RES_MS, KTX_MIN_EXP_RES_US);

                // Get info about the interface used to communicate with the camera.
                KTX_COMM_INTF interface = KTX_INTF_UNKNOWN;

                int32 interfaceId;
                pl_get_param(this->cameraContext->hcam, PARAM_CAM_INTERFACE_TYPE, ATTR_CURRENT,
                             (void *) &interfaceId);

                if (interfaceId == PL_CAM_IFC_TYPE_ETHERNET)
                    interface = KTX_INTF_ETHERNET;
                else if (interfaceId == PL_CAM_IFC_TYPE_VIRTUAL)
                    interface = KTX_INTF_VIRTUAL;
                else if (interfaceId == PL_CAM_IFC_TYPE_USB_1_1)
                    interface = KTX_INTF_USB_1_1;
                else if (interfaceId == PL_CAM_IFC_TYPE_USB_2_0)
                    interface = KTX_INTF_USB_2_0;
                else if (interfaceId == PL_CAM_IFC_TYPE_USB_3_0)
                    interface = KTX_INTF_USB_3_0;
                else if (interfaceId == PL_CAM_IFC_TYPE_USB_3_1)
                    interface = KTX_INTF_USB_3_1;
                else if (interfaceId == PL_CAM_IFC_TYPE_PCIE_X1)
                    interface = KTX_INTF_PCIE_x1;
                else if (interfaceId == PL_CAM_IFC_TYPE_PCIE_X4)
                    interface = KTX_INTF_PCIE_x4;
                else if (interfaceId == PL_CAM_IFC_TYPE_PCIE_X8)
                    interface = KTX_INTF_PCIE_x8;
                else if (interfaceId == PL_CAM_IFC_TYPE_PCIE)
                    interface = KTX_INTF_PCIE;

                setIntegerParam(KTX_CommInterface, interface);

                // Get information about connected camera resolution
                pl_get_param(this->cameraContext->hcam, PARAM_SER_SIZE, ATTR_CURRENT,
                             (void *) &this->cameraContext->sensorResX);
                pl_get_param(this->cameraContext->hcam, PARAM_PAR_SIZE, ATTR_CURRENT,
                             (void *) &this->cameraContext->sensorResY);

                setIntegerParam(ADMaxSizeX, this->cameraContext->sensorResX);
                setIntegerParam(ADMaxSizeY, this->cameraContext->sensorResY);
                INFO_ARGS("Model: %s | Resolution: %dx%d", modelStr,
                          this->cameraContext->sensorResX, this->cameraContext->sensorResY);

                INFO("Configuring default region to full sensor size...");
                setIntegerParam(ADMinX, 0);
                setIntegerParam(ADMinY, 0);
                setIntegerParam(ADBinX, 1);
                setIntegerParam(ADBinY, 1);
                setIntegerParam(ADSizeX, this->cameraContext->sensorResX);
                setIntegerParam(ADSizeY, this->cameraContext->sensorResY);
                setIntegerParam(NDColorMode,
                                NDColorModeMono);  // Only mono modes currently supported
                callParamCallbacks();
                updateCameraRegion();

                // Reset any pre-configured post-processing setup.
                pl_pp_reset(this->cameraContext->hcam);

                // Retrieve speed table information
                getSpeedTable();
                printSpeedTable();

                // Select default speed table mode
                setIntegerParam(KTX_ReadoutPortIdx, 0);
                setIntegerParam(KTX_SpeedIdx, 0);
                setIntegerParam(KTX_GainIdx, 0);
                selectSpeedTableMode();
                callParamCallbacks();

                // Register new frame callback function with PVCam SDK
                if (PV_OK !=
                    pl_cam_register_callback_ex3(this->cameraContext->hcam, PL_CALLBACK_EOF,
                                                 (void *) newFrameCallback, this->cameraContext)) {
                    ERR("Failed to register callback function!");
                } else {
                    INFO("Registered EOF callback function.");

                    // Spawn the monitoring thread. Make sure it's joinable.
                    INFO("Spawning camera monitor thread...");
                    this->monitoringActive = true;

                    epicsThreadOpts opts;
                    opts.priority = epicsThreadPriorityMedium;
                    opts.stackSize = epicsThreadGetStackSize(epicsThreadStackMedium);
                    opts.joinable = 1;

                    this->monitorThreadId = epicsThreadCreateOpt(
                        "monitorThread", (EPICSTHREADFUNC) monitorThreadC, this, &opts);
                }
            }
        }
    }
    epicsAtExit(exitCallbackC, (void *) this);
}

/**
 * @brief Function that runs temperature monitoring thread
 */
void ADKinetix::monitorThread() {
    const char *functionName = "monitorThread";

    INFO("Temperature monitor thread active.");

    while (this->monitoringActive) {
        // Only check temp if not acquiring
        if (!this->acquisitionActive) {
            this->pollingTemperature = true;
            int16 temperature = 0;
            bool ok = PV_OK == pl_get_param(this->cameraContext->hcam, PARAM_TEMP, ATTR_CURRENT,
                                            (void *) &temperature);
            // Clear before locking: acquisitionThread spins on this flag while holding the lock
            this->pollingTemperature = false;

            this->lock();
            if (!ok) {
                reportKinetixError(functionName);
            } else {
                setDoubleParam(KTX_Temperature, (double) (temperature / 100.0));
                callParamCallbacks();
            }
            this->unlock();
        }

        epicsThreadSleep(1);
    }
}

/**
 * @brief Starts acquisition by spawning main acq thread
 */
void ADKinetix::acquireStart() {
    const char *functionName = "acquireStart";

    // Spawn the acquisition thread. Make sure it's joinable.
    INFO("Spawning main acquisition thread...");

    epicsThreadOpts opts;
    opts.priority = epicsThreadPriorityHigh;
    opts.stackSize = epicsThreadGetStackSize(epicsThreadStackBig);
    opts.joinable = 1;

    this->acquisitionThreadId = epicsThreadCreateOpt(
        "acquisitionThread", (EPICSTHREADFUNC) acquisitionThreadC, this, &opts);
}

/**
 * @brief stops acquisition by aborting exposure and joinging acq thread
 */
void ADKinetix::acquireStop() {
    const char *functionName = "acquireStop";

    if (this->acquisitionActive) {
        // Abort current acquisition
        pl_exp_abort(this->cameraContext->hcam, CCS_HALT);

        // Mark acquisition as inactive
        this->acquisitionActive = false;

        // Acquisition thread needs the port lock to finish, so release it while joining
        INFO("Waiting for acquisition thread to join...");
        this->unlock();
        epicsThreadMustJoin(this->acquisitionThreadId);
        this->lock();
        INFO("Acquisition stopped.");

        // Refresh all PV values
        callParamCallbacks();
    } else {
        WARN("Acquisition is not active!");
    }
}

/**
 * @brief Updates descriptions of selected readout mode without applying it.
 */
void ADKinetix::updateReadoutPortDesc() {
    const char *functionName = "updateReadoutPortDesc";

    // Get readout port information
    int readoutPortIdx, speedIdx, gainIdx;
    getIntegerParam(KTX_ReadoutPortIdx, &readoutPortIdx);
    getIntegerParam(KTX_SpeedIdx, &speedIdx);
    getIntegerParam(KTX_GainIdx, &gainIdx);

    int valid = 1;

    if (this->cameraContext->speedTable.size() > (size_t) readoutPortIdx) {
        setStringParam(KTX_ReadoutPortDesc,
                       this->cameraContext->speedTable[readoutPortIdx].name.c_str());
    } else {
        setStringParam(KTX_ReadoutPortDesc, "Not Available");
        valid = 0;
    }
    if (valid == 1 &&
        this->cameraContext->speedTable[readoutPortIdx].speeds.size() > (size_t) speedIdx) {
        char speedDesc[40];
        snprintf(speedDesc, 40, "Pixel time: %d ns",
                 this->cameraContext->speedTable[readoutPortIdx].speeds[speedIdx].pixTimeNs);
        setStringParam(KTX_SpeedDesc, speedDesc);
    } else {
        setStringParam(KTX_SpeedDesc, "Invalid");
        valid = 0;
    }
    if (valid == 1 &&
        this->cameraContext->speedTable[readoutPortIdx].speeds[speedIdx].gains.size() >
            (size_t) gainIdx) {
        char gainDescStr[40];
        snprintf(gainDescStr, 40, "%s, %d bpp",
                 this->cameraContext->speedTable[readoutPortIdx]
                     .speeds[speedIdx]
                     .gains[gainIdx]
                     .name.c_str(),
                 this->cameraContext->speedTable[readoutPortIdx]
                     .speeds[speedIdx]
                     .gains[gainIdx]
                     .bitDepth);
        setStringParam(KTX_GainDesc, gainDescStr);
    } else {
        setStringParam(KTX_GainDesc, "Invalid");
        valid = 0;
    }

    if (valid == 1) {
        setIntegerParam(KTX_ModeValid, 1);
    } else {
        WARN("Selected readout mode is not supported!");
        setIntegerParam(KTX_ModeValid, 0);
    }
    callParamCallbacks();
}

/**
 * @brief Main acquisition function for ADKinetix
 */
void ADKinetix::acquisitionThread() {
    const char *functionName = "acquisitionThread";
    int acquisitionMode, targetNumImages, collectedImages, triggerMode, stopAcqOnTO, waitForFrameTO,
        minExpRes;
    bool eofSuccess;
    double exposureTime;
    int16 pvcamExposureMode;
    NDArray *pArray;
    NDArrayInfo arrayInfo;
    FRAME_INFO *pFrameInfo = nullptr;
    int32 lastFrameNr = -1;
    int32 frameNr = 0;
    uns32 pendingFrames = 0;
    int droppedFrames = 0;
    int uniqueIdSource;
    int circBuffSize;

    // Retrieve current frame information
    uns32 frameBufferSize;
    size_t dims[2];
    NDColorMode_t colorMode = NDColorModeMono;  // only grayscale at the moment
    NDDataType_t dataType;

    this->lock();
    getIntegerParam(NDDataType, (int *) &dataType);

    getCurrentFrameDimensions(dims);

    if (!this->speedTableModeSelected) {
        ERR("No speed table mode selected! Please check readout mode settings!");
        this->acquisitionActive = false;
        setIntegerParam(ADStatus, ADStatusError);
        setIntegerParam(ADAcquire, 0);
        callParamCallbacks();
        this->unlock();
        return;
    }

    // Retrieve any variable settings from PVs
    getDoubleParam(ADAcquireTime, &exposureTime);
    getIntegerParam(ADTriggerMode, &triggerMode);
    getIntegerParam(ADImageMode, &acquisitionMode);
    getIntegerParam(ADNumImages, &targetNumImages);
    getIntegerParam(KTX_StopAcqOnTimeout, &stopAcqOnTO);
    getIntegerParam(KTX_WaitForFrameTimeout, &waitForFrameTO);
    getIntegerParam(KTX_MinExpRes, &minExpRes);
    getIntegerParam(KTX_UniqueIdSource, &uniqueIdSource);
    getIntegerParam(KTX_CircBuffSize, &circBuffSize);

    // Convert EPICS selected trigger mode to PvCam format
    if (triggerMode == KTX_TRIG_INTERNAL)
        pvcamExposureMode = EXT_TRIG_INTERNAL | EXPOSE_OUT_FIRST_ROW;
    else if (triggerMode == KTX_TRIG_EDGE)
        pvcamExposureMode = EXT_TRIG_EDGE_RISING | EXPOSE_OUT_FIRST_ROW;
    else
        pvcamExposureMode = EXT_TRIG_LEVEL | EXPOSE_OUT_FIRST_ROW;

    // On overrun PVCam still overwrites the oldest frame, but get_oldest_frame_ex then returns an error
    const int16 circBuffMode = CIRC_NO_OVERWRITE;
    const bool continuousMode = (acquisitionMode != ADImageSingle);

    this->acquisitionActive = true;

    // Spin in place a bit to make sure the temp thread is not actively polling the camera
    while (this->pollingTemperature)
        ;

    INFO("Acquisition thread active.");

    INFO("Resetting image counter...");
    setIntegerParam(ADNumImagesCounter, 0);
    setIntegerParam(KTX_DroppedFrames, 0);
    setIntegerParam(KTX_PendingFrames, 0);
    callParamCallbacks();

    // Discard any EOF events left over from a previous acquisition
    {
        std::lock_guard<std::mutex> lock(this->cameraContext->eofEvent.mutex);
        this->cameraContext->pendingFrameNrs.clear();
        this->cameraContext->threadAbortFlag = false;
    }

    // In single mode, use exposure setup API call
    if (acquisitionMode == ADImageSingle) {
        pl_exp_setup_seq(this->cameraContext->hcam, 1, 1, &this->cameraContext->region,
                         pvcamExposureMode, (uns32) (exposureTime * pow(1000, minExpRes)),
                         &frameBufferSize);

        // Allocate frame buffer
        this->frameBuffer = (uns8 *) calloc(1, (size_t) frameBufferSize);
        INFO_ARGS("Allocated frame buffer of size %d...", frameBufferSize);

        if (PV_OK != pl_exp_start_seq(this->cameraContext->hcam, this->frameBuffer)) {
            reportKinetixError(functionName);
            this->acquisitionActive = false;
        }
    } else {
        // In continuous mode set up cont acq w/ circ buffer
        pl_exp_setup_cont(this->cameraContext->hcam, 1, &this->cameraContext->region,
                          pvcamExposureMode, (uns32) (exposureTime * pow(1000, minExpRes)),
                          &frameBufferSize, circBuffMode);

        // Allocate entire circular buffer; PVCam takes its total size as uns32
        uint64_t circBuffBytes = (uint64_t) circBuffSize * frameBufferSize;
        if (circBuffSize < 1 || circBuffBytes > UINT32_MAX) {
            ERR_ARGS("Invalid circular buffer size of %d frames (%llu bytes)! Can be at most %u bytes.", circBuffSize,
                     (unsigned long long) circBuffBytes, UINT32_MAX);
            this->frameBuffer = nullptr;
        } else {
            this->frameBuffer = (uns8 *) calloc((size_t) circBuffSize, (size_t) frameBufferSize);
        }

        if (!this->frameBuffer) {
            ERR("Failed to allocate circular buffer!");
            setIntegerParam(ADStatus, ADStatusError);
            this->acquisitionActive = false;
        } else if (PV_OK != pl_create_frame_info_struct(&pFrameInfo)) {
            reportKinetixError(functionName);
            this->acquisitionActive = false;
        } else {
            INFO_ARGS("Allocated circular buffer for %d frames. Total size: %llu bytes.",
                      circBuffSize, (unsigned long long) circBuffBytes);
            if (PV_OK != pl_exp_start_cont(this->cameraContext->hcam, this->frameBuffer,
                                           (uns32) circBuffBytes)) {
                reportKinetixError(functionName);
                this->acquisitionActive = false;
            }
        }
    }

    // In external trigger mode, set detector status to waiting before we get first frame
    if (triggerMode != KTX_TRIG_INTERNAL)
        setIntegerParam(ADStatus, ADStatusWaiting);
    else
        setIntegerParam(ADStatus, ADStatusAcquire);
    callParamCallbacks();

    while (acquisitionActive) {
        getIntegerParam(ADNumImagesCounter, &collectedImages);

        // Wait for new frame EOF event notification from callback
        this->unlock();
        eofSuccess = this->waitForEofEvent((uns32) waitForFrameTO, &pendingFrames, &frameNr);
        this->lock();
        setIntegerParam(KTX_PendingFrames, (int) pendingFrames);
        if (eofSuccess) {
            void *pFrame = this->frameBuffer;
            if (continuousMode) {
                if (PV_OK != pl_exp_get_oldest_frame_ex(this->cameraContext->hcam, &pFrame,
                                                        pFrameInfo)) {
                    // Expected after acquireStop() aborts the acquisition
                    if (this->acquisitionActive) reportKinetixError(functionName);
                    // EOF count no longer matches buffered frames; resync on the next EOF
                    {
                        std::lock_guard<std::mutex> lock(this->cameraContext->eofEvent.mutex);
                        this->cameraContext->pendingFrameNrs.clear();
                    }
                    setIntegerParam(KTX_PendingFrames, 0);
                    callParamCallbacks();
                    continue;
                }

                // get_oldest_frame_ex fills FRAME_INFO for the newest frame, so use the EOF callback's FrameNr
                if (lastFrameNr >= 0 && frameNr > lastFrameNr + 1) {
                    int missed = frameNr - lastFrameNr - 1;
                    droppedFrames += missed;
                    setIntegerParam(KTX_DroppedFrames, droppedFrames);
                    WARN_ARGS("Dropped %d frame(s) between frame %d and frame %d", missed,
                              lastFrameNr, frameNr);
                }
                lastFrameNr = frameNr;
            }

            // Allocate the NDArray of the correct size
            this->pArrays[0] = pNDArrayPool->alloc(2, dims, dataType, 0, NULL);
            if (this->pArrays[0] != NULL) {
                pArray = this->pArrays[0];
            } else {
                ERR("Failed to allocate array!");
                setIntegerParam(ADStatus, ADStatusError);
                if (acquisitionMode == ADImageSingle) {
                    pl_exp_finish_seq(this->cameraContext->hcam, this->frameBuffer, 0);
                } else {
                    pl_exp_unlock_oldest_frame(this->cameraContext->hcam);
                    pl_exp_abort(this->cameraContext->hcam, CCS_HALT);
                }
                this->acquisitionActive = false;
                callParamCallbacks();
                break;
            }

            collectedImages += 1;
            // New frame successfully collected.
            setIntegerParam(ADNumImagesCounter, collectedImages);
            updateTimeStamp(&pArray->epicsTS);

            // Set array size PVs based on collected frame
            pArray->getInfo(&arrayInfo);
            setIntegerParam(NDArraySize, (int) arrayInfo.totalBytes);
            setIntegerParam(NDArraySizeX, arrayInfo.xSize);
            setIntegerParam(NDArraySizeY, arrayInfo.ySize);

            setIntegerParam(ADStatus, ADStatusAcquire);

            // Copy the frame, then release its circular buffer slot back to PVCam
            memcpy(pArray->pData, pFrame, arrayInfo.totalBytes);
            if (continuousMode && PV_OK != pl_exp_unlock_oldest_frame(this->cameraContext->hcam))
                reportKinetixError(functionName);

            // increment the array counter
            int arrayCounter;
            getIntegerParam(NDArrayCounter, &arrayCounter);
            arrayCounter++;
            setIntegerParam(NDArrayCounter, arrayCounter);

            // set the image unique ID to either the camera frame number or the array counter
            pArray->uniqueId =
                (uniqueIdSource == KTX_UID_ARRAY_COUNTER) ? arrayCounter : (int) frameNr;

            pArray->pAttributeList->add("ColorMode", "Color Mode", NDAttrInt32, &colorMode);

            // Sends image to the ArrayDataPV
            getAttributes(pArray->pAttributeList);

            // Perform callbacks on collected frame; plugins must not run under the driver lock
            this->unlock();
            doCallbacksGenericPointer(pArray, NDArrayData, 0);
            this->lock();

            // If in single mode, finish acq, if in multiple mode and reached target number
            // complete acquisition.
            if (acquisitionMode == ADImageSingle) {
                pl_exp_finish_seq(this->cameraContext->hcam, this->frameBuffer, 0);
                this->acquisitionActive = false;
            } else if (acquisitionMode == ADImageMultiple && collectedImages == targetNumImages) {
                pl_exp_abort(this->cameraContext->hcam, CCS_HALT);
                this->acquisitionActive = false;
            }
            // Release the array
            pArray->release();
        } else if (stopAcqOnTO == 0) {
            // if we don't want to stop acq on a timeout, continue but set the status to waiting.
            WARN("Timeout waiting for next frame, trying again!");
            setIntegerParam(ADStatus, ADStatusWaiting);
        } else {
            // if we kill acq on TO, set status to aborting.
            setIntegerParam(ADStatus, ADStatusAborting);
            ERR("Failed to register EOF Event before timeout expired!");
            if (acquisitionMode == ADImageSingle)
                pl_exp_finish_seq(this->cameraContext->hcam, this->frameBuffer, 0);
            else
                pl_exp_abort(this->cameraContext->hcam, CCS_HALT);
            this->acquisitionActive = false;
        }
        // refresh all PVs
        callParamCallbacks();
    }

    INFO("Acquisition done. Freeing framebuffers.");
    if (pFrameInfo != nullptr) pl_release_frame_info_struct(pFrameInfo);
    free(this->frameBuffer);

    setIntegerParam(ADStatus, ADStatusIdle);
    setIntegerParam(ADAcquire, 0);
    setIntegerParam(KTX_PendingFrames, 0);
    callParamCallbacks();
    this->unlock();
}

/**
 * @brief Function for setting minimum exposure resolution
 *
 * @param currentExpRes Current minimum exposure resolution
 * @param newExpRes Target new minimum exposure resolution
 * @return asynSuccess if min exposure res was accepted, asynError otherwise.
 */
asynStatus ADKinetix::setMinExpRes(KTX_MIN_EXP_RES currentExpRes, KTX_MIN_EXP_RES newExpRes) {
    const char *functionName = "setMinExpRes";
    asynStatus status = asynSuccess;

    // Convert EPICS min exp res to pvcam type
    int32 minExpRes;
    switch (newExpRes) {
        case KTX_MIN_EXP_RES_S:
            minExpRes = EXP_RES_ONE_SEC;
            break;
        case KTX_MIN_EXP_RES_MS:
            minExpRes = EXP_RES_ONE_MILLISEC;
            break;
        case KTX_MIN_EXP_RES_US:
            minExpRes = EXP_RES_ONE_MICROSEC;
            break;
    }

    // Attempt to set requested minimum exposure resolution
    if (PV_OK != pl_set_param(this->cameraContext->hcam, PARAM_EXP_RES, (void *) &minExpRes)) {
        ERR_ARGS("Failed to configure minimum exposure resolution to %f second(s)!",
                 (double) (1 / pow(1000.0, (double) newExpRes)));
        setIntegerParam(KTX_MinExpRes, currentExpRes);
        status = asynError;
    } else {
        INFO_ARGS("Set minimum exposure time to %f second(s)!",
                  (double) (1 / pow(1000.0, (double) newExpRes)));
        setIntegerParam(KTX_MinExpRes, newExpRes);
    }
    callParamCallbacks();
    return status;
}

asynStatus ADKinetix::readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[],
                               size_t nElements, size_t *nIn) {
    int function = pasynUser->reason;
    const char *functionName = "readEnum";

    NVPC table;
    uns32 paramId;
    const char *paramName = NULL;

    *nIn = 0;

    if (function == KTX_FanSpeed) {
        paramId = PARAM_FAN_SPEED_SETPOINT;
        paramName = "PARAM_FAN_SPEED_SETPOINT";
        INFO("Reading possible fan speed states...");
    }
    if (paramName != NULL &&
        readEnumeration(this->cameraContext->hcam, &table, paramId, paramName)) {
        *nIn = table.size();
        for (size_t i = 0; i < *nIn; i++) {
            INFO_ARGS("Found possible state: %s", table[i].name.c_str());
            strings[i] = epicsStrDup(table[i].name.c_str());
            values[i] = table[i].value;
            severities[i] = 0;
        }
        return asynSuccess;
    }
    return asynError;
}

/**
 * @brief Override of ADDriver function - performs callbacks on write events to int PVs
 *
 * @param pasynUser Pointer to record asynUser instance
 * @param value Value written to PV
 * @returns asynSuccess if write was successful, asynError otherwise
 */
asynStatus ADKinetix::writeInt32(asynUser *pasynUser, epicsInt32 value) {
    const char *functionName = "writeInt32";
    int function = pasynUser->reason;
    int currentExpRes, acquiring, prevCircBuffSize;
    asynStatus status = asynSuccess;

    // Need to know current exp res when switching in case we cannot update
    getIntegerParam(KTX_MinExpRes, &currentExpRes);
    getIntegerParam(KTX_CircBuffSize, &prevCircBuffSize);

    // Certain actions depend on whether or not we are currently acquiring
    getIntegerParam(ADAcquire, &acquiring);

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we
     * read back the status at the end, but that's OK */
    status = setIntegerParam(function, value);

    if (function == ADAcquire) {
        if (acquiring == 1 && value == 0) {
            this->acquireStop();
        } else if (acquiring == 0 && value == 1) {
            this->acquireStart();
        } else if (value == 0) {
            ERR("Acquisition not active!");
            status = asynError;
        } else {
            ERR("Acquisition already active!");
            status = asynError;
        }
    } else if (function == KTX_GainIdx || function == KTX_SpeedIdx ||
               function == KTX_ReadoutPortIdx) {
        updateReadoutPortDesc();
    } else if (function == KTX_ApplyReadoutMode) {
        INFO("Applying selected readout mode...");
        int readoutModeValid;
        getIntegerParam(KTX_ModeValid, &readoutModeValid);
        if (readoutModeValid != 1) {
            ERR("Configured readout mode invalid!");
            status = asynError;
        } else {
            selectSpeedTableMode();
        }
    } else if (function == KTX_MinExpRes) {
        status = setMinExpRes((KTX_MIN_EXP_RES) currentExpRes, (KTX_MIN_EXP_RES) value);
    } else if (function == KTX_FanSpeed) {
        if (acquiring == 0) {
            if (PV_OK != pl_set_param(this->cameraContext->hcam, PARAM_FAN_SPEED_SETPOINT,
                                      (void *) &value)) {
                reportKinetixError(functionName);
                status = asynError;
            } else {
                setStringParam(ADStatusMessage, "Updated fan speed");
            }
        } else {
            ERR("Changing fan speed while acquiring is not permitted!");
        }
    } else if (function == KTX_CircBuffSize) {
        if (acquiring) {
            ERR("Changing circular buffer size while acquiring is not permitted!");
            setIntegerParam(KTX_CircBuffSize, prevCircBuffSize);
            status = asynError;
        } else if (value < 1) {
            ERR_ARGS("Circular buffer size must be at least 1 frame, got %d!", value);
            setIntegerParam(KTX_CircBuffSize, prevCircBuffSize);
            status = asynError;
        }
    } else if (function == ADBinX || function == ADBinY || function == ADMinX ||
               function == ADMinY || function == ADSizeX || function == ADSizeY) {
        // If we change binning or image dims, update the internal region state
        updateCameraRegion();
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if (status != asynSuccess) {
        ERR_ARGS("error, status=%d function=%d, value=%d", status, function, value);
    } else {
        DEBUG_ARGS("function=%d, value=%d", function, value);
    }

    return status;
}

/**
 * @brief Override of ADDriver function - performs callbacks on write events to flaot PVs
 *
 * @param pasynUser Pointer to record asynUser instance
 * @param value Value written to PV
 * @returns asynSuccess if write was successful, asynError otherwise
 */
asynStatus ADKinetix::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
    const char *functionName = "writeFloat64";
    int function = pasynUser->reason;
    int status = asynSuccess;

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we
     * read back the status at the end, but that's OK */
    status = setDoubleParam(function, value);

    if (function == ADAcquireTime) {
        // if exposure time is set to something longer than acquire period, change acquire period to
        // be the same
        double period;
        getDoubleParam(ADAcquirePeriod, &period);
        if (value > period) {
            setDoubleParam(ADAcquirePeriod, value);
        }
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if (status) {
        ERR_ARGS("error, status=%d function=%d, value=%f", status, function, value);
    } else {
        DEBUG_ARGS("function=%d, value=%f", function, value);
    }

    return (asynStatus) status;
}

/**
 * @brief Print information about driver to file
 *
 * @param fp Open file pointer (stdout)
 * @param details Level of details to print.
 */
void ADKinetix::report(FILE *fp, int details) {
    const char *functionName = "report";

    DEBUG("Logging driver information...");

    fprintf(fp, "Kinetix %s\n", this->portName);
    if (details > 0) {
        int nx, ny, dataType;
        getIntegerParam(ADSizeX, &nx);
        getIntegerParam(ADSizeY, &ny);
        getIntegerParam(NDDataType, &dataType);
        fprintf(fp, "  NX, NY:            %d  %d\n", nx, ny);
        fprintf(fp, "  Data type:         %d\n", dataType);
    }

    /* Invoke the base class method */
    ADDriver::report(fp, details);
}

/**
 * @brief ADKinetix destructor, called on exit to cleanup spawned threads.
 */
ADKinetix::~ADKinetix() {
    const char *functionName = "~ADKinetix";

    // Stop acquisiton if active; acquireStop expects the port lock to be held
    this->lock();
    this->acquireStop();
    this->unlock();

    // shut down separate threads
    this->monitoringActive = false;
    printf("Shutting down monitor thread...\n");
    epicsThreadMustJoin(this->monitorThreadId);

    // close camera if open
    if (this->cameraContext != NULL && this->cameraContext->isCamOpen) {
        printf("Closing camera...\n");
        pl_cam_close(this->cameraContext->hcam);
    }

    // delete camera context object
    delete this->cameraContext;

    // uninitialize SDK
    printf("Uninitializing SDK...\n");
    if (!pl_pvcam_uninit()) reportKinetixError(functionName);
    printf("Done.\n");
}

/**
 * @brief Helper function for extracting error message from code.
 *
 * @param functionName Name of PVCam SDK function
 */
void ADKinetix::reportKinetixError(const char *functionName) {
    int16 code;
    char error[256];

    code = pl_error_code();
    pl_error_message(code, error);
    ERR_ARGS("Error Code %d | %s", code, error);
    setStringParam(ADStatusMessage, error);
    setIntegerParam(ADStatus, ADStatusError);
    callParamCallbacks();
}

/* Code for iocsh registration */

/* ADKinetixConfig */
static const iocshArg ADKinetixConfigArg0 = {"DeviceIndex", iocshArgInt};
static const iocshArg ADKinetixConfigArg1 = {"Port name", iocshArgString};
static const iocshArg *const ADKinetixConfigArgs[] = {&ADKinetixConfigArg0, &ADKinetixConfigArg1};

static const iocshFuncDef configKinetix = {"ADKinetixConfig", 2, ADKinetixConfigArgs};

static void configKinetixCallFunc(const iocshArgBuf *args) {
    ADKinetixConfig(args[0].ival, args[1].sval);
}

static void kinetixRegister(void) { iocshRegister(&configKinetix, configKinetixCallFunc); }

extern "C" {
epicsExportRegistrar(kinetixRegister);
}
