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
		FreespaceDevice(OSVR_PluginRegContext ctx, FreespaceTracker* trackerInst) {
			/// Create the initialization options
			OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

			/// Assign the FreespaceTracker
			m_freespaceTracker = trackerInst;

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
					//m_freespaceTracker->handleLinkStatus(s.linkStatus);
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
					m_freespaceTracker->GetOrientation(yaw, pitch, roll, s.userFrame);
					trackerOrientationState = convEulerToQuat(yaw, pitch, roll);
					osvrDeviceTrackerSendOrientation(m_dev, m_tracker, &trackerOrientationState, 0);
					break;
				default:
					std::cout << "Received an unhandled message from freespace device. " << std::endl;
					break;
				}
			}

			return OSVR_RETURN_SUCCESS;
		}
		void deviceSetConfiguration(bool send_body_frames, bool send_user_frames)
		{
			_sendBodyFrames = send_body_frames;
			_sendUserFrames = send_user_frames;
		}
	private:
		osvr::pluginkit::DeviceToken m_dev;
		OSVR_TrackerDeviceInterface m_tracker;
		osvr::util::time::TimeValue m_timestamp;
		FreespaceDeviceInfo _deviceInfo;
		FreespaceTracker* m_freespaceTracker;
		double m_myVal;
		bool _sendBodyFrames = false;
		bool _sendUserFrames = true;
		float _lastBodyFrameTime;
		

		
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
			/* translation = { 0, 0, 0 };
			OSVR_Quaternion rotation = { w, y, z, x };
			OSVR_PoseState trackerCoords = 
			{
				translation,
				rotation
			};*/

			return trackerCoords;
		}
	};

	class HardwareDetection {
	public:
		HardwareDetection() : m_found(false) {}
		OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {

			/*std::cout << "PLUGIN: Got a hardware detection request" << std::endl;
			if (!m_found) {
				std::cout << "PLUGIN: We have detected our Freespace device! Doing "
					"setup stuff!" << std::endl;
				m_found = true;

				/// Create our device object, passing the context
				osvr::pluginkit::registerObjectForDeletion(ctx, new FreespaceDevice(ctx, deviceIndex++));
			}
			return OSVR_RETURN_SUCCESS;*/
			std::cout << "PLUGIN: Got a hardware detection request" << std::endl;
			if (!m_found) {
			// create a freespace tracker for the first device.
				m_freespaceTracker = FreespaceTracker::create("FreespaceDevice", 0, false, true);
			if (!m_freespaceTracker) {
				std::cout << "PLUGIN: Error opening freespace device : FreespaceDevice"  << std::endl;
				return OSVR_RETURN_FAILURE;
			}

			
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
		int deviceIndex = 0;
		FreespaceTracker* m_freespaceTracker;
	};

	class FreespaceTracker {
	public:
		FreespaceDeviceId _freespaceDevice;
		FreespaceDeviceInfo _deviceInfo;

		static FreespaceTracker* create(const char *name,
			int device_index = 0,
			bool send_body_frames = false,
			bool send_user_frames = true)
		{
			// initialize libfreespace
			FreespaceTracker::freespaceInit();

			const unsigned MAX_DEVICES = 100;
			FreespaceDeviceId devices[MAX_DEVICES];
			int numIds = 0;
			int rc;
			rc = freespace_getDeviceList(devices, MAX_DEVICES, &numIds);
			if ((rc != FREESPACE_SUCCESS) || (numIds < (device_index + 1))) {
				std::cout << "vrpn_Freespace::create: Didn't find enough devices: " << numIds << ".\n" << std::endl;
				return nullptr;
			}
			FreespaceDeviceId freespaceId = devices[device_index];

			rc = freespace_openDevice(freespaceId);
			if (rc != FREESPACE_SUCCESS) {
				std::cout << "vrpn_Freespace::create: Could not open device " << device_index << ".\n" << std::endl;
				return nullptr;
			}
			struct FreespaceDeviceInfo deviceInfo;
			rc = freespace_getDeviceInfo(freespaceId, &deviceInfo);
			if (rc != FREESPACE_SUCCESS) {
				return nullptr;
			}

			// printf("\nFreespace Device Info:\n----------------------\nDevice = %s\nVendor ID = 0x%x (%d) \nProduct ID = 0x%x (%d)\n\n", deviceInfo.name, deviceInfo.vendor, deviceInfo.product, deviceInfo.product);

			rc = freespace_flush(freespaceId);
			if (rc != FREESPACE_SUCCESS) {
				std::cout << "freespaceInputThread: Error flushing device:" << rc << ".\n" << std::endl;
				return nullptr;
			}

			FreespaceTracker* dev = new FreespaceTracker(freespaceId, &deviceInfo, name);
			//dev->deviceSetConfiguration(send_body_frames, send_user_frames);
			std::cout << "Added freespace device:" << name << "\n" << std::endl;
			return dev;
		}
		~FreespaceTracker()
		{
			// as part of cleanup, disable user and body frame messages, and
			// put back to just mouse movement.
			int rc = freespace_flush(_freespaceDevice);
			if (rc != FREESPACE_SUCCESS) {
				printf("freespaceInputThread: Error flushing device: %d\n", rc);
			}

			deviceUnconfigure();
			freespace_closeDevice(_freespaceDevice);
		}
		void deviceConfigure()
		{
			int rc = 0;
			int deviceType = 0;

			// Send the data mode request
			struct freespace_message message;
			memset(&message, 0, sizeof(message));

			switch (_deviceInfo.product) {
			case FSM6_PID:
				message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
				message.dataModeControlV2Request.packetSelect = 8; // MotionEngine Output
				message.dataModeControlV2Request.mode = 4; // Full Motion On
				message.dataModeControlV2Request.formatSelect = 0; // Format 0
				if (_sendBodyFrames) {
					message.dataModeControlV2Request.ff1 = 1; // Enable Linear Acceleration
					message.dataModeControlV2Request.ff3 = 1; // Enable Angular Velocity
					message.dataModeControlV2Request.ff0 = 1; // Enable cursor and buttons
				}
				if (_sendUserFrames) {
					message.dataModeControlV2Request.ff6 = 1; // Enable Angular Position
					message.dataModeControlV2Request.ff0 = 1; // Enable cursor and buttons
				}
				break;
			case FSM9_PID:
				message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
				message.dataModeControlV2Request.packetSelect = 8; // MotionEngine Output
				message.dataModeControlV2Request.mode = 4; // Full Motion On
				message.dataModeControlV2Request.formatSelect = 0; // Format 0
				if (_sendBodyFrames) {
					message.dataModeControlV2Request.ff1 = 1; // Enable Linear Acceleration
					message.dataModeControlV2Request.ff3 = 1; // Enable Angular Velocity
					message.dataModeControlV2Request.ff0 = 1; // Enable cursor and buttons
				}
				if (_sendUserFrames) {
					message.dataModeControlV2Request.ff6 = 1; // Enable Angular Position
					message.dataModeControlV2Request.ff0 = 1; // Enable cursor and buttons
				}
				break;
			case SCOOP_PID:
				//Send V2 request.
				message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
				message.dataModeControlV2Request.packetSelect = 4;
				if (_sendBodyFrames) {
					if (_sendUserFrames) {
						message.dataModeControlV2Request.packetSelect = 4; // BodyUserFrame Message
					}
					else {
						message.dataModeControlV2Request.packetSelect = 2; // BodyFrame Message
					}
				}
				else if (_sendUserFrames) {
					message.dataModeControlV2Request.packetSelect = 3; // UserFrame Message
				}
				message.dataModeControlV2Request.mode = 4;
				break;
			default:
				printf("Unsupported Freespace device\n");
				break;
			}

			/*
			if (_deviceInfo.hVer == 1) {
			message.messageType = FREESPACE_MESSAGE_DATAMOTIONCONTROL;
			struct freespace_DataMotionControl * req;
			req = &message.dataMotionControl;
			if (_sendBodyFrames) { req->enableBodyMotion = 1; }
			if (_sendUserFrames) { req->enableUserPosition = 1; }
			req->inhibitPowerManager = 1;
			req->enableMouseMovement = 0;
			req->disableFreespace = 0;
			} else {
			message.messageType = FREESPACE_MESSAGE_DATAMODEREQUEST;
			struct freespace_DataModeRequest * req;
			req = &message.dataModeRequest;
			if (_sendBodyFrames) { req->enableBodyMotion = 1; }
			if (_sendUserFrames) { req->enableUserPosition = 1; }
			req->inhibitPowerManager = 1;
			req->enableMouseMovement = 0;
			req->disableFreespace = 0;
			}
			*/

			rc = freespace_sendMessage(_freespaceDevice, &message);
			if (rc != FREESPACE_SUCCESS) {
				fprintf(stderr, "vrpn_Freespace::deviceConfigure: Could not send message: %d.\n", rc);
			}
		}
	private:
		bool _sendBodyFrames;
		bool _sendUserFrames;
		float _lastBodyFrameTime;
		osvr::util::time::TimeValue _timestamp;
		static void freespaceInit() {
			static bool freespaceInit = false;
			if (!freespaceInit) {
				std::cout << "Initialized freespace lib " << std::endl;
				freespaceInit = true;
				int rc = freespace_init();
				if (rc != FREESPACE_SUCCESS) {
					std::cout << "failed to init freespace lib " << std::endl;
				}
			}
		}
		/**
		* private constructor since opening of the device can fail.
		*/
		FreespaceTracker(FreespaceDeviceId freespaceId, struct FreespaceDeviceInfo* deviceInfo, const char *name) :_freespaceDevice(freespaceId)
		{
			memcpy(&_deviceInfo, deviceInfo, sizeof(_deviceInfo));
			memset(&_lastBodyFrameTime, 0, sizeof(_lastBodyFrameTime));
			_timestamp.seconds = 0;
		}

		void GetOrientation(long &yaw, long &pitch, long &roll, const struct freespace_UserFrame& user)
		{
			// Get the quaternion vector
			long w = user.angularPosA;
			long x = user.angularPosB;
			long y = user.angularPosC;
			long z = user.angularPosD;

			// normalize the vector
			long len = sqrtl((w*w) + (x*x) + (y*y) + (z*z));
			w /= len;
			x /= len;
			y /= len;
			z /= len;

			// The Freespace quaternion gives the rotation in terms of
			// rotating the world around the object. We take the conjugate to
			// get the rotation in the object's reference frame.
			w = w;
			x = -x;
			y = -y;
			z = -z;

			// Convert to angles in radians
			long m11 = (2.0f * w * w) + (2.0f * x * x) - 1.0f;
			long m12 = (2.0f * x * y) + (2.0f * w * z);
			long m13 = (2.0f * x * z) - (2.0f * w * y);
			long m23 = (2.0f * y * z) + (2.0f * w * x);
			long m33 = (2.0f * w * w) + (2.0f * z * z) - 1.0f;

			// Just return the raw samples
			roll = atan2l(m23, m33);
			pitch = asinl(-m13);
			yaw = atan2l(m12, m11);

			std::cout << "Yaw: " << yaw << std::endl;
			std::cout << "Pitch: " << pitch << std::endl;
			std::cout << "Roll: " << roll << std::endl;
		}
		
		void deviceUnconfigure() {
			int rc = 0;

			// Send the data mode request
			struct freespace_message message;
			memset(&message, 0, sizeof(message));
			if (_deviceInfo.hVer == 1) {
				message.messageType = FREESPACE_MESSAGE_DATAMOTIONCONTROL;
				struct freespace_DataMotionControl * req;
				req = &message.dataMotionControl;
				req->enableBodyMotion = 0;
				req->enableUserPosition = 0;
				req->inhibitPowerManager = 0;
				req->enableMouseMovement = 1;
				req->disableFreespace = 0;
			}
			else {
				message.messageType = FREESPACE_MESSAGE_DATAMODEREQUEST;
				struct freespace_DataModeRequest * req;
				req = &message.dataModeRequest;
				req->enableBodyMotion = 0;
				req->enableUserPosition = 0;
				req->inhibitPowerManager = 0;
				req->enableMouseMovement = 1;
				req->disableFreespace = 0;
			}

			rc = freespace_sendMessage(_freespaceDevice, &message);
			if (rc != FREESPACE_SUCCESS) {
				fprintf(stderr, "vrpn_Freespace::deviceUnconfigure: Could not send message: %d.\n", rc);
			}
		}
	};
} // namespace

OSVR_PLUGIN(com_osvr_FSM9) {
	/// Register custom message type
	osvrDeviceRegisterMessageType(ctx, "FreespaceDevice", &freespaceMessage);

	osvr::pluginkit::PluginContext context(ctx);

	/// Register a detection callback function object.
	context.registerHardwareDetectCallback(new HardwareDetection());

	return OSVR_RETURN_SUCCESS;
}
