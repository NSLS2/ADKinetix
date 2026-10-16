#ifndef ADKINETIXDS_H
#define ADKINETIXDS_H

#include <pvcam.h>
#include <stddef.h>
#include <stdlib.h>

#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "master.h"

// Custom value used as a placeholder for an invalid post-processing parameter ID
constexpr uns32 cInvalidPpId = (uns32) -1;

/*
 * Common data types
 */

// Name-Value Pair type. Used in various places: when iterating PVCAM enumerable
// parameter types, or for storing GUI selection options.
struct NVP {
    int32 value{0};
    std::string name{};
};
// Name-Value Pair Container type. Holds a list of name-value pairs.
typedef std::vector<NVP> NVPC;

// The camera speed table forms a structure of a tree. Cameras usually have one or
// more 'ports', each port may have one or more 'speeds' (readout rates), and each
// speed may have one or more 'gains'

// This structure holds description of a gain entry.
struct SpdtabGain {
    // In PVCAM, gain indexes are 1-based.
    int32 index{1};
    // Not all cameras support named gains. If not supported, this
    // string stays empty.
    std::string name{};
    // The bit-depth may be different for each gain, therefore it is stored
    // within this structure. For example, the Prime BSI camera has gains
    // with different bit-depths under the same speed.
    int16 bitDepth{0};
};
// This structure holds description of a speed entry.
struct SpdtabSpeed {
    // In PVCAM, speed indexes are 0-based.
    int32 index{0};
    // Pixel time can be used to calculate the overall readout rate. This is less
    // relevant with sCMOS sensors, but the pix time is still reported to provide
    // an approximate readout rate of a particular speed.
    uns16 pixTimeNs{1};
    // List of gains under this particular speed.
    std::vector<SpdtabGain> gains;
};
// This structure holds description of a port entry.
struct SpdtabPort {
    // Please note that the PARAM_READOUT_PORT is an ENUM_TYPE parameter.
    // For this reason, the current port is not reported as an index, but
    // as a generic number. Applications that are written in a generic way
    // to support all Teledyne Photometrics cameras should not rely on a fact
    // that port numbers are usually zero-based.
    int32 value{0};
    // Name of this port, as retrieved by enumerating the PARAM_READOUT_PORT.
    std::string name{};
    // List of speeds under this particular port.
    std::vector<SpdtabSpeed> speeds;
};

// Platform-independent replacement for Windows event
struct Event {
    // Mutex that guards all other members
    std::mutex mutex{};
    // Condition that any thread could wait on
    std::condition_variable cond{};
    // A flag that helps with spurious wakeups
    bool flag{false};
};

// In PVCAM, post-processing parameters are a specific group of camera-reported
// parameters. The list of post processing 'features' and underlying 'parameters' needs
// to be retrieved dynamically after opening the camera. The structure of camera
// post-processing modes forms a tree structure where the camera usually provides
// a list of 'features'. Every 'feature' then contains a list of 'parameters'. To
// navigate in the feature/parameter tree, two parameters are used: PARAM_PP_INDEX
// (for feature traversing) and PARAM_PP_PARAM_INDEX (for parameter traversing under
// a particular feature). This is sufficient to display the tree structure in a UI.
// In order for the application to locate a specific PP parameter and set it programmatically,
// two other parameters can be used: PARAM_PP_FEAT_ID and PARAM_PP_PARAM_ID.
// A post-processing parameter itself is then accessed through the PARAM_PP_PARAM.
// This parameter is of an uns32 type, thus, essentially, all PP parameters are of
// the same type. If a boolean switch is reported as a PP parameter, it will usually
// be reported as a generic uns32 type with an allowed value range between 0 and 1.
// The structures below are used to cache the post-processing tree structure.

// PVCAM Post-Processing parameter descriptor
struct PvcamPpParameter {
    int16 index{-1};         // PARAM_PP_PARAM_INDEX
    uns32 id{cInvalidPpId};  // PARAM_PP_PARAM_ID
    std::string name{};      // PARAM_PP_PARAM_NAME
    uns32 minValue{0};       // PARAM_PP_PARAM, ATTR_MIN
    uns32 maxValue{0};       // PARAM_PP_PARAM, ATTR_MAX
    uns32 defValue{0};       // PARAM_PP_PARAM, ATTR_DEFAULT
};

// PVCAM Post-Processing feature descriptor
struct PvcamPpFeature {
    int16 index{-1};                                // PARAM_PP_INDEX
    uns32 id{cInvalidPpId};                         // PARAM_PP_FEAT_ID
    std::string name{};                             // PARAM_PP_FEAT_NAME
    std::vector<PvcamPpParameter> parameterList{};  // List of underlying parameters
};

// Structure holding camera-related properties, one instance per camera
struct CameraContext {
    // Camera name, the only member valid before opening camera
    char camName[CAM_NAME_LEN]{'\0'};

    // Set to true when PVCAM opens camera
    bool isCamOpen{false};

    // All members below are initialized once the camera is successfully open

    // Camera handle
    int16 hcam{-1};

    // Camera sensor serial size (sensor width)
    uns16 sensorResX{0};
    // Camera sensor parallel size (sensor height)
    uns16 sensorResY{0};
    // Sensor region and binning factors to be used for the acquisition,
    // initialized to full sensor size with 1x1 binning upon opening the camera.
    rgn_type region{0, 0, 0, 0, 0, 0};

    // Vector of camera readout options, commonly referred to as 'speed table'
    std::vector<SpdtabPort> speedTable{};

    // Image format reported after acq. setup, value from PL_IMAGE_FORMATS
    int32 imageFormat{PL_IMAGE_FORMAT_MONO16};
    // Sensor type (if not Frame Transfer CCD then camera is Interline CCD or sCMOS).
    // Not relevant for sCMOS sensors.
    bool isFrameTransfer{false};
    // Flag marking the camera as Smart Streaming capable
    bool isSmartStreaming{false};

    // Event used for communication between acq. loop and EOF callback routine
    Event eofEvent{};
    // FrameNr of each EOF event not yet consumed by the acquisition thread, guarded by eofEvent.mutex
    std::deque<int32> pendingFrameNrs{};
    // Storage for a code sample specific context data if needed for callback acquisition
    void* eofContext{nullptr};

    // Used as an acquisition thread or for other independent tasks
    std::thread* thread{nullptr};
    // Flag to be set to abort thread (used, for example, in multi-camera code samples)
    bool threadAbortFlag{false};
};

#endif
