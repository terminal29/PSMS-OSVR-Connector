/** @date 2016
	@author
	Sensics, Inc.
	<http://sensics.com/osvr>
*/

// Copyright 2016 Sensics Inc.
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

// Internal Includes
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>
#include <osvr/PluginKit/AnalogInterfaceC.h>
#include <osvr/PluginKit/ButtonInterfaceC.h>

// Generated JSON header file
#include "inf_psmove_osvr_connector_json.h"

// Standard includes
#include <chrono>
#include <thread>
#include <iostream>
#include <string>
#include <json/json.h>

// 3rd party includes
#include <PSMoveClient_CAPI.h>
#include <ClientGeometry_CAPI.h>

#define M_PI 3.14159265358979323846 

#define DEVICE_NAME "PSMSDevice"
#define CONSOLE_PREFIX "[PSMS OSVR Plugin]"

#define DEFAULT_TIMEOUT 2000

namespace {

	class PSMSDevice {
	public:
		PSMSDevice(OSVR_PluginRegContext ctx, int par_hmdControllerID, int par_leftHandControllerID, int par_rightHandControllerID) {
			// Create OSVR Device options
			OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

			// Set local vars with controller IDs
			// I know there's a more concise way to do this part
			hmdControllerID = par_hmdControllerID;
			leftControllerID = par_leftHandControllerID;
			rightControllerID = par_rightHandControllerID;

			// Set up all connected & Requested controllers
			if (hmdControllerID != -1) {
				controllers[0] = PSM_GetController(hmdControllerID);
				PSM_AllocateControllerListener(hmdControllerID);
				PSM_StartControllerDataStream(hmdControllerID, PSMStreamFlags_includePositionData | PSMStreamFlags_includeCalibratedSensorData, 1000);
			}
			if (leftControllerID != -1) {
				controllers[1] = PSM_GetController(leftControllerID);
				PSM_AllocateControllerListener(leftControllerID);
				PSM_StartControllerDataStream(leftControllerID, PSMStreamFlags_includePositionData | PSMStreamFlags_includeCalibratedSensorData, 1000);
			}
			if (rightControllerID != -1) {
				controllers[2] = PSM_GetController(rightControllerID);
				PSM_AllocateControllerListener(rightControllerID);
				PSM_StartControllerDataStream(rightControllerID, PSMStreamFlags_includePositionData | PSMStreamFlags_includeCalibratedSensorData, 1000);
			}

			// configure our tracker
			osvrDeviceTrackerConfigure(opts, &m_tracker);

			//Configure button steam. 
			//osvrDeviceButtonConfigure(opts, &m_buttons, 1);

			// Create the device token with the options
			m_dev.initAsync(ctx, DEVICE_NAME, opts);

			// Send JSON descriptor
			m_dev.sendJsonDescriptor(inf_psmove_osvr_connector_json);

			// Register update callback
			m_dev.registerUpdateCallback(this);

		}

		OSVR_ReturnCode update() {
			PSM_UpdateNoPollMessages();
			//TODO Check that the controllers are still tracking, otherwise recycle last frame's pose

			if (hmdControllerID != -1) {
				c_poses[0] = PSMtoOSVRPoseState(&(controllers[0]->ControllerState.PSMoveState.Pose));
				osvrDeviceTrackerSendPose(m_dev, m_tracker, &c_poses[0], 0);
			}
			if (leftControllerID != -1) {
				c_poses[1] = PSMtoOSVRPoseState(&(controllers[1]->ControllerState.PSMoveState.Pose));
				osvrDeviceTrackerSendPose(m_dev, m_tracker, &c_poses[1], 1);
			}
			if (rightControllerID != -1) {
				c_poses[2] = PSMtoOSVRPoseState(&(controllers[2]->ControllerState.PSMoveState.Pose));
				osvrDeviceTrackerSendPose(m_dev, m_tracker, &c_poses[2], 2);
			}

			return OSVR_RETURN_SUCCESS;
		}

	private:
		osvr::pluginkit::DeviceToken m_dev;
		OSVR_TrackerDeviceInterface m_tracker;
		OSVR_ButtonDeviceInterface m_buttons;

		//OSVR Poses for each controller
		OSVR_PoseState c_poses[3];

		//PSMoveService Controller Handles
		PSMController *controllers[3] = { nullptr, nullptr, nullptr };
		int hmdControllerID = -1;
		int leftControllerID = -1;
		int rightControllerID = -1;

		OSVR_PoseState PSMtoOSVRPoseState(const PSMPosef *psmPose) {
			OSVR_PoseState pstate;
			osvrQuatSetIdentity(&(pstate.rotation));
			osvrVec3Zero(&(pstate.translation));

			// To convert PSMS Meters into OSVR Centimeters
			osvrVec3SetX(&(pstate.translation), (psmPose->Position.x / 100));
			osvrVec3SetY(&(pstate.translation), (psmPose->Position.y / 100));
			osvrVec3SetZ(&(pstate.translation), (psmPose->Position.z / 100));

			// Just pass straight through for now
			osvrQuatSetW(&(pstate.rotation), psmPose->Orientation.w); //w
			osvrQuatSetX(&(pstate.rotation), psmPose->Orientation.x); //x
			osvrQuatSetY(&(pstate.rotation), psmPose->Orientation.y); //y
			osvrQuatSetZ(&(pstate.rotation), psmPose->Orientation.z); //z

			return pstate;
		}
	};
class PSMS_OSVR_Constructor {
public:
	OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx, const char *params) {

		// Connect to PSMS
		std::cout << CONSOLE_PREFIX << " Connecting to PSMoveService" << std::endl;

		// Connect to the PSM Server (0 - found psmove service, anything else error)
		int connectResult = PSM_Initialize(PSMOVESERVICE_DEFAULT_ADDRESS, PSMOVESERVICE_DEFAULT_PORT, DEFAULT_TIMEOUT);

		if (connectResult != 0) {
			std::cout << CONSOLE_PREFIX << " Could not connect to PSMoveService, make sure to start it before OSVR." << std::endl;
			return OSVR_RETURN_FAILURE;
		}

		// Read the JSON data from parameters.
		Json::Value root;
		if (params) {
			Json::Reader r;
			if (!r.parse(params, root)) {
				std::cout << CONSOLE_PREFIX << ": Could not parse config parameters!" << std::endl;
				return OSVR_RETURN_FAILURE;
			}
		}

		// Get requested controller IDs
		int controllerIDs[3] = {
			root.get("hmdController", -1).asInt(), // HMD
			root.get("leftHandController", -1).asInt(), //Left Hand
			root.get("rightHandController", -1).asInt() //Right Hand
		};
		std::string controllerDesc[3] = { "HMD","Left Hand","Right Hand" };
		bool doesControllerExist[3] = { false, false, false };
		PSMControllerList controllerList;
		
		PSM_GetControllerList(&controllerList, DEFAULT_TIMEOUT);
		// Check controllers exist before sending to our device. If they don't, reset back to -1 so our device ignores them
		// For each controller
		for (int i = 0; i < 3; i++) {
			//Check if it matches an ID that is connected (if not -1 already)
			for (int k = 0; k < controllerList.count; k++) {
				//Found our controller connected!
				if (controllerIDs[i] != -1 && controllerList.controller_id[k] == controllerIDs[i]) {
					std::cout << CONSOLE_PREFIX << " Connected controller " << controllerIDs[i] << " with tracker for " << controllerDesc[i] << std::endl;
					doesControllerExist[i] = true;
					break;
				}
			}
			// Cant find our controller connected!
			if (!doesControllerExist[i]) {
				std::cout << CONSOLE_PREFIX << " Designated controller with Controller ID " << controllerIDs[i] << " does not seem to be connected..." << std::endl;
			}
		}
		
		osvr::pluginkit::registerObjectForDeletion(ctx, new PSMSDevice(ctx, controllerIDs[0], controllerIDs[1], controllerIDs[2]));

		return OSVR_RETURN_SUCCESS;
		}
	
	};

} // namespace

OSVR_PLUGIN(inf_psmove_osvr_connector) {

    osvr::pluginkit::PluginContext context(ctx);
	osvr::pluginkit::registerDriverInstantiationCallback(ctx, DEVICE_NAME, new PSMS_OSVR_Constructor);

    return OSVR_RETURN_SUCCESS;
}