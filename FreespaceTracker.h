/** @file
    @brief Header

    @date 2015

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2015 Vuzix Corporation.
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

#ifndef INCLUDED_FreespaceTracker_h_GUID_9253C2FC_831E_4D97_45C7_A681F310C5D8
#define INCLUDED_FreespaceTracker_h_GUID_9253C2FC_831E_4D97_45C7_A681F310C5D8

#define SCOOP_PID 0xC0B3
#define FSM6_PID 0xC080
#define FSM9_PID 0xC0E0

// Internal Includes
// - none

// Library/third-party includes
#include <math.h>
#include "freespace/freespace.h"
#pragma comment ( lib, "Setupapi.lib")
#pragma comment ( lib, "hid.lib")

// Standard includes
#include <iostream>

namespace {
class FreespaceTracker {
	static void hotplug_Callback(enum freespace_hotplugEvent evnt, FreespaceDeviceId id, void* params) {
		FreespaceTracker* sensor = (FreespaceTracker*)params;

		printf("Freespace hot plug callback triggered");

		if (evnt == FREESPACE_HOTPLUG_REMOVAL) {
			//sensor->_removeDevice(id);
		}
		else if (evnt == FREESPACE_HOTPLUG_INSERTION) {
			//sensor->_initDevice(id);
		}
	}
	
  public:
	  

	  const unsigned MAX_DEVICES = 100;
	  FreespaceDeviceId devices[100];
	  int numIds;
	  FreespaceTracker(const char *name,
		  int device_index = 0,
		  bool send_body_frames = false,
		  bool send_user_frames = true)
	  {
		  //create//
		  // initialize libfreespace
		  freespaceInit();

		  int rc;
		  std::cout << "before " << numIds << ".\n" << std::endl;
		  rc = freespace_getDeviceList(devices, MAX_DEVICES, &numIds);
		  std::cout << "numids " << numIds << ".\n" << std::endl;
		  if ((rc != FREESPACE_SUCCESS) || (numIds < (device_index + 1))) {

			  std::cout << "FreespaceTracker::FreespaceTracker: Didn't find enough devices: " << numIds << ".\n" << std::endl;
		  }
		  FreespaceDeviceId freespaceId = devices[device_index];

		  rc = freespace_openDevice(freespaceId);
		  if (rc != FREESPACE_SUCCESS) {
			  std::cout << "FreespaceTracker::FreespaceTracker: Could not open device " << device_index << ".\n" << std::endl;
		  }
		  struct FreespaceDeviceInfo deviceInfo;
		  rc = freespace_getDeviceInfo(freespaceId, &deviceInfo);
		  if (rc != FREESPACE_SUCCESS) {
			  std::cout << "FreespaceTracker::FreespaceTracker: Could not get device info " << device_index << ".\n" << std::endl;
		  }

		  // printf("\nFreespace Device Info:\n----------------------\nDevice = %s\nVendor ID = 0x%x (%d) \nProduct ID = 0x%x (%d)\n\n", deviceInfo.name, deviceInfo.vendor, deviceInfo.product, deviceInfo.product);

		  rc = freespace_flush(freespaceId);
		  if (rc != FREESPACE_SUCCESS) {
			  std::cout << "freespaceInputThread: Error flushing device:" << rc << ".\n" << std::endl;
		  }

		  //endcreate//
		  _freespaceDevice = freespaceId;
		  std::cout << "freespacetracker constructor.\n" << std::endl;
		  memcpy(&_deviceInfo, &deviceInfo, sizeof(_deviceInfo));
		  memset(&_lastBodyFrameTime, 0, sizeof(_lastBodyFrameTime));
		  _timestamp.seconds = 0;
		  deviceSetConfiguration(send_body_frames, send_user_frames);

		  if (rc == FREESPACE_SUCCESS && !_initialized)
		  {
			  std::cout << "Added freespace device:" << name << "\n" << std::endl;
			  _initialized = true;
		  }
		  else
		  {
			  std::cout << "Added freespace device:" << name << "\n" << std::endl;
			  _initialized = false;
		  }
	  }

	  //tell freespace t unit
    ~FreespaceTracker() 
	{
		// as part of cleanup, disable user and body frame messages, and
		// put back to just mouse movement.
		int rc = freespace_flush(_freespaceDevice);
		if (rc != FREESPACE_SUCCESS) {
			std::cout << "freespaceInputThread: Error flushing device:" << rc << ".\n" <<std::endl;
		}

		deviceUnconfigure();
		freespace_closeDevice(_freespaceDevice);
		freespace_exit();
	}

	void GetOrientation(long &yaw, long &pitch, long &roll, const struct freespace_UserFrame& user)
	{
		// Get the quaternion vector
		float w = user.angularPosA;
		float x = user.angularPosB;
		float y = user.angularPosC;
		float z = user.angularPosD;

		// normalize the vector
		float len = sqrtf((w*w) + (x*x) + (y*y) + (z*z));
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
		float m11 = (2.0f * w * w) + (2.0f * x * x) - 1.0f;
		float m12 = (2.0f * x * y) + (2.0f * w * z);
		float m13 = (2.0f * x * z) - (2.0f * w * y);
		float m23 = (2.0f * y * z) + (2.0f * w * x);
		float m33 = (2.0f * w * w) + (2.0f * z * z) - 1.0f;

		// Just return the raw samples
		roll = atan2f(m23, m33);
		pitch = asinf(-m13);
		yaw = atan2f(m12, m11);
	}

	//void handleBodyFrame(const struct freespace_BodyFrame&);
	//void handleBodyUserFrame(const struct freespace_BodyUserFrame&);
	void handleLinkStatus(const struct freespace_LinkStatus&)
	{
		// Could be used to send messages when the loop is powered off/unavailable.
	}
	void handleMotionEngineOutput(const struct freespace_MotionEngineOutput&);

	void deviceSetConfiguration(bool send_body_frames, bool send_user_frames)
	{
		_sendBodyFrames = send_body_frames;
		_sendUserFrames = send_user_frames;
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
	void deviceUnconfigure()
	{
		int rc = 0;

		// Send the data mode request
		struct freespace_message message;
		memset(&message, 0, sizeof(message));

		switch (_deviceInfo.product) {
		case SCOOP_PID:
			message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
			message.dataModeControlV2Request.packetSelect = 1; // Mouse mode
			message.dataModeControlV2Request.mode = 0; // Full motion
			break;
		case FSM6_PID:
			message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
			message.dataModeControlV2Request.packetSelect = 1; // Mouse mode
			message.dataModeControlV2Request.mode = 0; // Full motion
			break;
		case FSM9_PID:
			message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
			message.dataModeControlV2Request.packetSelect = 0; // None
			message.dataModeControlV2Request.mode = 1; // Sleep
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
		req->enableBodyMotion = 0;
		req->enableUserPosition = 0;
		req->inhibitPowerManager = 0;
		req->enableMouseMovement = 1;
		req->disableFreespace = 0;
		} else {
		message.messageType = FREESPACE_MESSAGE_DATAMODEREQUEST;
		struct freespace_DataModeRequest * req;
		req = &message.dataModeRequest;
		req->enableBodyMotion = 0;
		req->enableUserPosition = 0;
		req->inhibitPowerManager = 0;
		req->enableMouseMovement = 1;
		req->disableFreespace = 0;
		}
		*/

		rc = freespace_sendMessage(_freespaceDevice, &message);
		if (rc != FREESPACE_SUCCESS) {
			fprintf(stderr, "vrpn_Freespace::deviceUnconfigure: Could not send message: %d.\n", rc);
		}
	}

	FreespaceDeviceId _freespaceDevice;
	FreespaceDeviceInfo _deviceInfo;
	bool _initialized = false;

	private:
		/*FreespaceTracker* create(const char *name,
			int device_index = 0,
			bool send_body_frames = false,
			bool send_user_frames = true)
		{
			// initialize libfreespace
			freespaceInit();

			const unsigned MAX_DEVICES = 100;
			FreespaceDeviceId devices[MAX_DEVICES];
			int numIds = 0;
			int rc;
			std::cout << "before " << numIds << ".\n" << std::endl;
			rc = freespace_getDeviceList(devices, MAX_DEVICES, &numIds);
			std::cout << "numids " << numIds << ".\n" << std::endl;
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
			dev->deviceSetConfiguration(send_body_frames, send_user_frames);
			std::cout << "Added freespace device:" << name << "\n" << std::endl;
			return dev;
		}*/
		static void freespaceInit() {
			static bool freespaceInit = false;
			if (!freespaceInit) {
				freespaceInit = true;
				int rc = freespace_init();
				if (rc != FREESPACE_SUCCESS) {
					std::cout << "vrpn_Freespace::freespaceInit: failed to init freespace lib. rc= " << rc << ".\n" << std::endl;
					freespaceInit = false;
				}
			}
		}
		
		
		bool _sendBodyFrames;
		bool _sendUserFrames;
		osvr::util::time::TimeValue _timestamp;
		int _deviceCount = 0;

	protected:
	  float _lastBodyFrameTime;
};
} // namespace

#endif // INCLUDED_FreespaceTracker_h_GUID_9253C2FC_831E_4D97_45C7_A681F310C5D8
