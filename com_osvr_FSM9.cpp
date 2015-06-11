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
#include "FreespaceTracker.h"

// Generated JSON header file
#include "com_osvr_FSM9_json.h"

// Library/third-party includes
#include <math.h>

// Standard includes
#include <iostream>
#include <memory>


// Anonymous namespace to avoid symbol collision
namespace {

class FreespaceDevice {
	
  public:
	  FreespaceDevice(OSVR_PluginRegContext ctx, FreespaceTracker* trackerInst) {
        /// Create the initialization options
        OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);
		
		/// Assign the FreespaceTracker
		m_freespaceTracker = trackerInst;

        /// Indicate that we'll report tracking
        osvrDeviceTrackerConfigure(opts, &m_tracker);

        /// Create the sync device token with the options
        m_dev.initSync(ctx, "FreespaceDevice", opts);

        /// Send JSON descriptor
        m_dev.sendJsonDescriptor(com_osvr_FSM9_json);

        /// Register update callback
        m_dev.registerUpdateCallback(this);
    }

    OSVR_ReturnCode update() {

		struct freespace_message s;
		/*
		* Send configuration information to the device repeatedly to force the
		* device into the correct mode.  This method ensures the correct mode
		* even if the Freespace device was asleep during initialization or if
		* the tracker resets.
		*/
		osvr::util::time::TimeValue timestamp;

		osvrTimeValueGetNow(&timestamp);
		if (m_timestamp.seconds != timestamp.seconds) {
			m_timestamp.seconds = timestamp.seconds;
			m_freespaceTracker->deviceConfigure();
		}
		long yaw, pitch, roll;
		OSVR_OrientationState trackerOrientationState;
		// Do not block, read as many messages as 
		while (FREESPACE_SUCCESS == freespace_readMessage(m_freespaceTracker->_freespaceDevice, &s, 0)) {
			switch (s.messageType) {
			case FREESPACE_MESSAGE_LINKSTATUS:
				std::cout << "FREESPACE_MESSAGE_LINKSTATUS " << std::endl;
				m_freespaceTracker->handleLinkStatus(s.linkStatus);
				break;
			case FREESPACE_MESSAGE_BODYFRAME:
				std::cout << "FREESPACE_MESSAGE_BODYFRAME " << std::endl;
				//m_freespaceTracker->handleBodyFrame(s.bodyFrame);
				break;
			case FREESPACE_MESSAGE_USERFRAME:
				std::cout << "FREESPACE_MESSAGE_USERFRAME " << std::endl;
				m_freespaceTracker->GetOrientation(yaw, pitch, roll, s.userFrame);
				trackerOrientationState = convEulerToQuat(yaw, pitch, roll);
				osvrDeviceTrackerSendOrientation(m_dev, m_tracker, &trackerOrientationState, 0);
				return OSVR_RETURN_SUCCESS;
			case FREESPACE_MESSAGE_ALWAYSONRESPONSE:
				std::cout << "FREESPACE_MESSAGE_ALWAYSONRESPONSE " << std::endl;
				break;
			case FREESPACE_MESSAGE_DATAMODERESPONSE:
				std::cout << "FREESPACE_MESSAGE_DATAMODERESPONSE " << std::endl;
				break;
			case FREESPACE_MESSAGE_PRODUCTIDRESPONSE:
				std::cout << "FREESPACE_MESSAGE_PRODUCTIDRESPONSE " << std::endl;
				break;
			case FREESPACE_MESSAGE_DATAMODECONTROLV2RESPONSE:
				std::cout << "FREESPACE_MESSAGE_DATAMODECONTROLV2RESPONSE " << std::endl;
				break;
			case FREESPACE_MESSAGE_MOTIONENGINEOUTPUT:
				std::cout << "FREESPACE_MESSAGE_MOTIONENGINEOUTPUT " << std::endl;
				//handleMotionEngineOutput(s.motionEngineOutput);
				break;
			default:
				std::cout << "Received an unhandled message from freespace device. " << std::endl;
				break;
			}
		}

		return OSVR_RETURN_SUCCESS;
    }
	static OSVR_OrientationState convEulerToQuat(long yaw, long pitch,
		long roll) {

		const double rawToRad = M_PI / 32768.0f;

		// convert raw values to radians
		double yawRad = yaw * rawToRad;
		double pitchRad = pitch * rawToRad;
		double rollRad = roll * rawToRad;
		// invert roll to correct rolling direction
		rollRad *= -1.0;

		/* put angles into radians and divide by two, since all angles in
		* formula
		*  are (angle/2)
		*/

		double half_yaw = yawRad / 2.0;
		double half_pitch = pitchRad / 2.0;
		double half_roll = rollRad / 2.0;

		double cosYaw = cos(half_yaw);
		double sinYaw = sin(half_yaw);

		double cosPitch = cos(half_pitch);
		double sinPitch = sin(half_pitch);

		double cosRoll = cos(half_roll);
		double sinRoll = sin(half_roll);

		double x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
		double y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
		double z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;

		double w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;

		OSVR_OrientationState trackerCoords = { w, y, z, x };

		return trackerCoords;
	}



  private:
    osvr::pluginkit::DeviceToken m_dev;
    OSVR_TrackerDeviceInterface m_tracker;
    FreespaceTracker* m_freespaceTracker;
	osvr::util::time::TimeValue m_timestamp;
	
};

class HardwareDetection {
  public:
    HardwareDetection() : m_found(false){}
    OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {

        std::cout << "PLUGIN: Got a hardware detection request" << std::endl;

		// create a freespace tracker for the first device.
		printf("Tracker's name is %s.\n", TRACKER_NAME);
		m_freespaceTracker = FreespaceTracker::create(TRACKER_NAME, 0, false, true);
		if (!m_freespaceTracker) {
			fprintf(stderr, "Error opening freespace device: %s\n", TRACKER_NAME);
			return OSVR_RETURN_FAILURE;
		}
		
		if (!m_found) {
			std::cout << "PLUGIN: We have detected a Freespace device! " << std::endl;
			m_found = true;
			/// Create our device object
			osvr::pluginkit::registerObjectForDeletion(ctx, new FreespaceDevice(ctx, m_freespaceTracker));
		}
		return OSVR_RETURN_SUCCESS;
    }

  private:
    /// @brief Have we found our device yet? (this limits the plugin to one
    /// instance)
    bool m_found;
	char *TRACKER_NAME = "Freespace0";
	FreespaceTracker* m_freespaceTracker;
};
} // namespace

OSVR_PLUGIN(com_osvr_FSM9) {
    osvr::pluginkit::PluginContext context(ctx);

    /// Register a detection callback function object.
    context.registerHardwareDetectCallback(new HardwareDetection());

    return OSVR_RETURN_SUCCESS;
}
