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

// Internal Includes
// Internal Includes
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>

// Generated JSON header file
#include "com_osvr_Freespace_json.h"

// Library/third-party includes
#include <math.h>

// Standard includes
#include <iostream>
#include <memory>


// Anonymous namespace to avoid symbol collision
namespace {
OSVR_MessageType dummyMessage;
class FreespaceDevice {
  public:
	  FreespaceDevice(OSVR_PluginRegContext ctx) {
        /// Create the initialization options
        OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

        /// Indicate that we'll report tracking
        osvrDeviceTrackerConfigure(opts, &m_tracker);

        /// Create the sync device token with the options
        m_dev.initAsync(ctx, "FreespaceDevice", opts);

        /// Send JSON descriptor
        m_dev.sendJsonDescriptor(com_osvr_Freespace_json);

        /// Register update callback
        m_dev.registerUpdateCallback(this);
    }

	/// In this case, the core spawns a thread, with a loop calling this
	/// function as long as things are running. So this function waits for the
	/// next message from the device and passes it on.
    OSVR_ReturnCode update() {
		/// Make up some dummy data that changes to report.
		m_myVal = (m_myVal + 0.1);
		if (m_myVal > 10.0) {
			m_myVal = 0;
		}
		/// Report the identity pose for sensor 0
		OSVR_OrientationState pose = { m_myVal, 0, 0, 1 };
		osvrDeviceTrackerSendOrientation(m_dev, m_tracker, &pose, 0);
		return OSVR_RETURN_SUCCESS;
    }
  private:
    osvr::pluginkit::DeviceToken m_dev;
    OSVR_TrackerDeviceInterface m_tracker;
	osvr::util::time::TimeValue m_timestamp;
	double m_myVal;
	
};

class HardwareDetection {
  public:
	 HardwareDetection() : m_found(false) {}
    OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {

        std::cout << "PLUGIN: Got a hardware detection request" << std::endl;
        if (!m_found) {
            std::cout << "PLUGIN: We have detected our Freespace device! Doing "
                         "setup stuff!" << std::endl;
            m_found = true;

            /// Create our device object, passing the context
            osvr::pluginkit::registerObjectForDeletion(ctx, new FreespaceDevice(ctx));
        }
        return OSVR_RETURN_SUCCESS;
    }

  private:
    /// @brief Have we found our device yet? (this limits the plugin to one
    /// instance)
	  bool m_found;
};
} // namespace

OSVR_PLUGIN(com_osvr_FSM9) {
	/// Register custom message type
	osvrDeviceRegisterMessageType(ctx, "FreespaceDevice", &dummyMessage);

    osvr::pluginkit::PluginContext context(ctx);

    /// Register a detection callback function object.
    context.registerHardwareDetectCallback(new HardwareDetection());

    return OSVR_RETURN_SUCCESS;
}
