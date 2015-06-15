/** @file
@brief Comprehensive example: Implementation of a dummy Hardware Detect
Callback that creates a dummy device when it is "detected"

@date 2014

@author
Sensics, Inc.
<http://sensics.com/osvr>
*/

// Copyright 2014 Sensics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// Needed to get M_PI from math.h on Windows.
#define _USE_MATH_DEFINES

#define SCOOP_PID 0xC0B3
#define FSM6_PID 0xC080
#define FSM9_PID 0xC0E0

// Internal Includes
// Internal Includes
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>

// Generated JSON header file
#include "com_osvr_Freespace_json.h"

// Library/third-party includes
#include <math.h>
#include "freespace/freespace.h"
#include <freespace/freespace_util.h>
#ifdef _WIN32
#pragma comment(lib, "Setupapi.lib")
#pragma comment(lib, "hid.lib")
#endif

// Standard includes
#include <iostream>
#include <memory>
#include <set>
#include <array>
#include <assert.h>

// Anonymous namespace to avoid symbol collision
namespace {

/// @brief Simple class holding the lifetime of the freespace lib
class FreespaceLib {
  public:
    FreespaceLib() {
        auto ret = freespace_init();
        if (ret != FREESPACE_SUCCESS) {
            throw std::runtime_error(
                "Failed to initialize libfreespace, code was " +
                std::to_string(ret));
        }
    }
    ~FreespaceLib() { freespace_exit(); }
};

/// @brief RAII wrapper for opening a libfreespace device
class FreespaceDeviceIdWrapper {
  public:
    /// @brief Default constructor
    FreespaceDeviceIdWrapper() = default;

    /// @brief Constructor from ID - tries to open the device.
    explicit FreespaceDeviceIdWrapper(FreespaceDeviceId id) : m_id(id) {
        auto ret = freespace_openDevice(m_id);
        m_valid = (FREESPACE_SUCCESS == ret);
        if (!m_valid) {
            std::cout << "Error in freespace_openDevice, code " << ret
                      << std::endl;
        }
    }

    /// @brief Move constructor
    FreespaceDeviceIdWrapper(FreespaceDeviceIdWrapper &&other)
        : m_id(other.m_id), m_valid(other.m_valid) {
        other.m_valid = false;
    }

    /// @brief move assignable
    FreespaceDeviceIdWrapper &operator=(FreespaceDeviceIdWrapper &&other) {
        close();
        m_id = other.m_id;
        m_valid = other.m_valid;
        other.m_valid = false;
        return *this;
    }

    /// @brief Destructor
    ~FreespaceDeviceIdWrapper() { close(); }

    /// @brief non-copyable
    FreespaceDeviceIdWrapper(FreespaceDeviceIdWrapper const &other) = delete;

    /// @brief not copy assignable
    FreespaceDeviceIdWrapper &
    operator=(FreespaceDeviceIdWrapper const &other) = delete;

    /// @brief Check validity/"is open"
    explicit operator bool() const { return m_valid; }

    /// @brief Get the device ID
    FreespaceDeviceId get() const {
        assert(m_valid &&
               "get() only makes sense if the ID is actually validly opened");
        return m_id;
    }

    /// @brief Close device manually
    void close() {
        if (m_valid) {
            freespace_closeDevice(m_id);
            m_valid = false;
        }
    }

  private:
    FreespaceDeviceId m_id;
    bool m_valid = false;
};

class FreespaceDevice {
  public:
    static const unsigned int READ_TIMEOUT_MS = 100;
    FreespaceDevice(OSVR_PluginRegContext ctx, FreespaceDeviceIdWrapper &&devId,
                    const char *name)
        : m_deviceId(std::move(devId)) {

        /// Create the initialization options
        OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

        /// Indicate that we'll report tracking.
        osvrDeviceTrackerConfigure(opts, &m_tracker);

        /// Create an asynchronous (threaded) device
        m_dev.initAsync(ctx, name, opts);

        /// Send JSON descriptor
        m_dev.sendJsonDescriptor(com_osvr_Freespace_json);

        printf("Registering update callback deviceId=%d\n", m_deviceId.get());
        /// Sets the update callback
        m_dev.registerUpdateCallback(this);
    }

    /// In this case, the core spawns a thread, with a loop calling this
    /// function as long as things are running. So this function waits for the
    /// next message from the device and passes it on.
    OSVR_ReturnCode update() {
        struct freespace_message message;
        auto ret =
            freespace_readMessage(m_deviceId.get(), &message, READ_TIMEOUT_MS);
        if (ret == FREESPACE_ERROR_TIMEOUT ||
            ret == FREESPACE_ERROR_INTERRUPTED) {
            // Both timeout and interrupted are ok.
            // Timeout happens if there aren't any events for a second.
            // Interrupted happens if you type CTRL-C or if you
            // type CTRL-Z and background the app on Linux.
            return OSVR_RETURN_SUCCESS;
        }
        if (ret != FREESPACE_SUCCESS) {
            // This means we didn't have success and we didn't have an excusable
            // error.
            // Close communications with the device
            m_cleanup();
            return OSVR_RETURN_FAILURE;
        }

        // freespace_printMessage(stdout, &message); // This just prints the
        // basic message fields
        if (message.messageType != FREESPACE_MESSAGE_MOTIONENGINEOUTPUT) {
            printf("freespace OSVR plugin: unhandled message ignored of type: "
                   "%d\n",
                   message.messageType);
            return OSVR_RETURN_SUCCESS;
        }

        struct MultiAxisSensor angPos;
        auto getRet =
            freespace_util_getAngPos(&message.motionEngineOutput, &angPos);
        if (getRet != 0) {
            printf(
                "freespace OSVR plugin: freespace_util_getAngPos returned %d\n",
                getRet);
            return OSVR_RETURN_SUCCESS; // pretend it's harmless.
        }

        OSVR_OrientationState orientation;
        osvrQuatSetW(&orientation, angPos.w);
        osvrQuatSetX(&orientation, angPos.x);
        osvrQuatSetY(&orientation, angPos.y);
        osvrQuatSetZ(&orientation, angPos.z);
        osvrDeviceTrackerSendOrientation(m_dev, m_tracker, &orientation, 0);

        return OSVR_RETURN_SUCCESS;
    }

  private:
    void m_cleanup() { m_deviceId.close(); }
    osvr::pluginkit::DeviceToken m_dev;
    OSVR_TrackerDeviceInterface m_tracker;
    FreespaceDeviceIdWrapper m_deviceId;
};

inline std::vector<FreespaceDeviceId> getDeviceList() {
    std::vector<FreespaceDeviceId> devices;
    int numFound = 0;
    std::array<FreespaceDeviceId, FREESPACE_MAXIMUM_DEVICE_COUNT> devicesRaw;
    auto ret = freespace_getDeviceList(devicesRaw.data(), devicesRaw.size(),
                                       &numFound);
    if (ret == FREESPACE_SUCCESS && numFound > 0) {
        devices.assign(devicesRaw.data(), devicesRaw.data() + numFound);
    }
    return devices;
}

/**
* printDeviceInfo
* Common helper function that prints the information about a device
*
* @param id The ID of the device to print the info for.
* @param FREESPACE_SUCCESS or an error
*/
int printDeviceInfo(FreespaceDeviceId id) {
    struct FreespaceDeviceInfo info;
    int rc;

    // Retrieve the information for the device
    rc = freespace_getDeviceInfo(id, &info);
    if (rc != FREESPACE_SUCCESS) {
        return rc;
    }

    printf("    Device = %s\n    Vendor ID  = 0x%x (%d)\n    Product ID = "
           "0x%x (%d)\n",
           info.name, info.vendor, info.vendor, info.product, info.product);

    return FREESPACE_SUCCESS;
}

class HardwareDetection {
  public:
    HardwareDetection() {}

    FreespaceDeviceIdWrapper attemptDeviceSetup(FreespaceDeviceId id) const {
        FreespaceDeviceIdWrapper retVal;
        std::cout << "ID " << id;
        if (m_handledDevices.find(id) != m_handledDevices.end()) {
            std::cout << " - Already handled" << std::endl;
            return retVal;
        }
        std::cout << " - Not yet handled." << std::endl;
        // Display the device information.

        auto ret = printDeviceInfo(id);
        if (ret != FREESPACE_SUCCESS) {
            std::cout << " - Couldn't get device info! skipping..."
                      << std::endl;
            return retVal;
        }

        printf("\nTrying to open device id %d\n", id);
        // Prepare to communicate with the device found above

        FreespaceDeviceIdWrapper dev(id);
        if (!dev) {
            return dev;
        }

        // Make sure any old messages are cleared out of the system
        ret = freespace_flush(dev.get());
        if (ret != FREESPACE_SUCCESS) {
            printf("Error flushing device: code %d\n", ret);
            return retVal;
        }

        // Configure the device for motion outputs
        printf("Sending message to enable motion data.\n");
        struct freespace_message message = {0};

        message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
        message.dataModeControlV2Request.packetSelect =
            8;                                     // MotionEngine Output
        message.dataModeControlV2Request.mode = 4; // Full Motion On
        message.dataModeControlV2Request.formatSelect = 0; // Format 0

        message.dataModeControlV2Request.ff6 = 1; // Enable Angular Position
        message.dataModeControlV2Request.ff0 = 1; // Enable cursor and buttons

        ret = freespace_sendMessage(dev.get(), &message);
        if (ret != FREESPACE_SUCCESS) {
            printf("Could not send message: code %d.\n", ret);
            return retVal;
        }

        retVal = std::move(dev);

        return retVal;
    }
    OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {
        std::cout << "PLUGIN: Got a hardware detection request" << std::endl;

        std::cout << "Scanning for Freespace devices" << std::endl;
        std::vector<FreespaceDeviceId> devices = getDeviceList();
        std::cout << "Got " << devices.size() << " devices" << std::endl;

        for (auto id : devices) {
            auto newDevice = attemptDeviceSetup(id);
            if (!newDevice) {
                continue;
            }
            std::string name = "Freespace";
            name += std::to_string(m_handledDevices.size());
            osvr::pluginkit::registerObjectForDeletion(
                ctx,
                new FreespaceDevice(ctx, std::move(newDevice), name.c_str()));
            m_handledDevices.insert(id);
        }
        return OSVR_RETURN_SUCCESS;
    }

  private:
    std::set<FreespaceDeviceId> m_handledDevices;
};

} // namespace

OSVR_PLUGIN(com_osvr_Freespace) {
    std::cout << "com_osvr_Freespace: Plugin loaded, libfreespace version "
              << freespace_version() << std::endl;
    osvr::pluginkit::PluginContext context(ctx);

    try {
        std::unique_ptr<FreespaceLib> libHandle(new FreespaceLib());
        context.registerObjectForDeletion(libHandle.release());
    } catch (std::runtime_error &e) {
        std::cerr << "com_osvr_Freespace: " << e.what() << std::endl;
        return OSVR_RETURN_FAILURE;
    }
    /// Register a detection callback function object.
    context.registerHardwareDetectCallback(new HardwareDetection());

    return OSVR_RETURN_SUCCESS;
}
