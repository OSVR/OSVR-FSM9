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
#include "freespace/freespace.h";
#include <freespace/freespace_util.h>
#pragma comment ( lib, "Setupapi.lib")
#pragma comment ( lib, "hid.lib")

// Standard includes
#include <iostream>
#include <memory>


// Anonymous namespace to avoid symbol collision
namespace {
	OSVR_MessageType freespaceMessage;
	class FreespaceDevice {
	public:
		FreespaceDevice(OSVR_PluginRegContext ctx, FreespaceDeviceId deviceId) {
			/// Create the initialization options
			OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

			/// Indicate that we'll report tracking.
			osvrDeviceTrackerConfigure(opts, &m_tracker);

			/// Create an asynchronous (threaded) device
			m_dev.initAsync(ctx, "Freespace", opts);

			/// Send JSON descriptor
			m_dev.sendJsonDescriptor(com_osvr_Freespace_json);

			_deviceId = deviceId;
			printf("Registering update callback deviceId=%d\n", _deviceId);
			/// Sets the update callback
			m_dev.registerUpdateCallback(this);
		}

		/// In this case, the core spawns a thread, with a loop calling this
		/// function as long as things are running. So this function waits for the
		/// next message from the device and passes it on.
		OSVR_ReturnCode update() {

			// A loop to read messages
			//printf("Listening for messages on device with id %d\n", _deviceId);

				rc = freespace_readMessage(_deviceId, &message, 100);
				if (rc == FREESPACE_ERROR_TIMEOUT ||
					rc == FREESPACE_ERROR_INTERRUPTED) {
					// Both timeout and interrupted are ok.
					// Timeout happens if there aren't any events for a second.
					// Interrupted happens if you type CTRL-C or if you
					// type CTRL-Z and background the app on Linux.
					//continue;
					//printf("FREESPACE_ERROR_TIMEOUT\n", rc);
				}
				if (rc != FREESPACE_SUCCESS) {
					// Close communications with the device
					printf("Cleaning up...\n");
					freespace_closeDevice(_deviceId);
					/** --- END EXAMPLE FINALIZATION OF DEVICE --- **/

					// Cleanup the library
					freespace_exit();
					return OSVR_RETURN_FAILURE;
					//break;
				}

				// freespace_printMessage(stdout, &message); // This just prints the basic message fields
				if (message.messageType == FREESPACE_MESSAGE_MOTIONENGINEOUTPUT) {
					//rc = freespace_util_getAngularVelocity(&message.motionEngineOutput, &angVel);
					rc = freespace_util_getAngPos(&message.motionEngineOutput, &angVel);
					if (rc == 0) {
						//printf("X: % 6.2f, Y: % 6.2f, Z: % 6.2f\n", angVel.x, angVel.y, angVel.z);
						OSVR_OrientationState orientation = { angVel.x, angVel.y, angVel.z, angVel.w };
						osvrDeviceTrackerSendOrientation(m_dev, m_tracker, &orientation, 0);
					}		
				}			

			return OSVR_RETURN_SUCCESS;
		}

	private:
		osvr::pluginkit::DeviceToken m_dev;
		OSVR_TrackerDeviceInterface m_tracker;
		//osvr::util::time::TimeValue m_timestamp;
		FreespaceDeviceInfo _deviceInfo;
		FreespaceDeviceId _deviceId;
		int rc;
		struct freespace_message message;
		struct MultiAxisSensor angVel;
	};
		
	class HardwareDetection {
	public:
		HardwareDetection() : deviceId(0){}
		OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {

				std::cout << "PLUGIN: Got a hardware detection request" << std::endl;
				if (!m_found)
				{
					m_found = true;
					// Initialize the freespace library
					rc = freespace_init();
					if (rc != FREESPACE_SUCCESS) {
						printf("Initialization error. rc=%d\n", rc);
						return OSVR_RETURN_FAILURE;
					}
				}
				
			
				printf("Scanning for Freespace devices...\n");
				// Get the ID of the first device in the list of availble devices
				rc = freespace_getDeviceList(&deviceIds, FREESPACE_MAXIMUM_DEVICE_COUNT, &numIds);
				if (numIds == 0) {
					printf("Didn't find any devices.\n");
					return OSVR_RETURN_FAILURE;
				}
				printf("Found %d devices ", numIds);

				printf("Found a device with id %d. Trying to open it...\n", deviceId);
				// Prepare to communicate with the device found above
				rc = freespace_openDevice(deviceId);
				if (rc != FREESPACE_SUCCESS) {
					printf("Error opening device: %d\n", rc);
					return OSVR_RETURN_FAILURE;
				}
				// Display the device information.
				//printDeviceInfo(deviceId);

				// Make sure any old messages are cleared out of the system
				rc = freespace_flush(deviceId);
				if (rc != FREESPACE_SUCCESS) {
					printf("Error flushing device: %d\n", rc);
					return OSVR_RETURN_FAILURE;
				}

				// Configure the device for motion outputs
				printf("Sending message to enable motion data.\n");
				memset(&message, 0, sizeof(message)); // Make sure all the message fields are initialized to 0.

				/*message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
				message.dataModeControlV2Request.packetSelect = 8;  // MotionEngine Outout
				message.dataModeControlV2Request.mode = 0;          // Set full motion
				message.dataModeControlV2Request.formatSelect = 0;  // MEOut format 0
				message.dataModeControlV2Request.ff0 = 1;           // Pointer fields
				message.dataModeControlV2Request.ff3 = 1;           // Angular velocity fields*/
				message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
				message.dataModeControlV2Request.packetSelect = 8; // MotionEngine Output
				message.dataModeControlV2Request.mode = 4; // Full Motion On
				message.dataModeControlV2Request.formatSelect = 0; // Format 0
				if (false) {
					message.dataModeControlV2Request.ff1 = 1; // Enable Linear Acceleration
					message.dataModeControlV2Request.ff3 = 1; // Enable Angular Velocity
					message.dataModeControlV2Request.ff0 = 1; // Enable cursor and buttons
				}
				if (true) {
					message.dataModeControlV2Request.ff6 = 1; // Enable Angular Position
					message.dataModeControlV2Request.ff0 = 1; // Enable cursor and buttons
				}

				rc = freespace_sendMessage(deviceId, &message);
				if (rc != FREESPACE_SUCCESS) {
					printf("Could not send message: %d.\n", rc);
					return OSVR_RETURN_FAILURE;
				}
				printf("Registering object for deletion: %d.\n", deviceId);
				/// Create our device object, passing the context
				osvr::pluginkit::registerObjectForDeletion(ctx, new FreespaceDevice(ctx, deviceId));
				//deviceId++;
			return OSVR_RETURN_SUCCESS;
		}

	private:
		struct freespace_message message;
		FreespaceDeviceId deviceIds;
		FreespaceDeviceId deviceId;
		int numIds;	// The number of device Ids found
		int rc;		// Return Code
		bool m_found = false;
	};

} // namespace

OSVR_PLUGIN(com_osvr_Freespace) {
	/// Register custom message type
	//osvrDeviceRegisterMessageType(ctx, "FreespaceDevice", &freespaceMessage);

	osvr::pluginkit::PluginContext context(ctx);

	/// Register a detection callback function object.
	context.registerHardwareDetectCallback(new HardwareDetection());

	return OSVR_RETURN_SUCCESS;
}
