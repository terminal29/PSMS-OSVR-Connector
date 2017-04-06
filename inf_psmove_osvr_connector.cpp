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
#include <boost/math/quaternion.hpp>
#include <boost/algorithm/string.hpp>


# define M_PI           3.14159265358979323846 
/*
Example config JSON file for pairing controllers with their osvr device.

{"controller_device_pairing":{[
		"hmd":"00:00:00:00:00",
		"lefthand" : "00:00:00:00:01",
		"righthand" : "00:00:00:00:02",
	]}
}
*/

//
#define PLUGIN_NAME "PSMS OSVR Plugin"

// Anonymous namespace to avoid symbol collision
namespace {

	class PSMSDevice {
	public:
		PSMSDevice(OSVR_PluginRegContext ctx, int par_hmdControllerID, int par_leftHandControllerID, int par_rightHandControllerID) {
			/// Create the initialization options
			OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

			// I know theres a better way to do this but i cant be bothered atm.
			hmdControllerID = par_hmdControllerID;
			leftControllerID = par_leftHandControllerID;
			rightControllerID = par_rightHandControllerID;
			std::cout << par_hmdControllerID << par_leftHandControllerID << par_rightHandControllerID << std::endl;

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
			
			// TODO Remember to add flags to data stream when including button presses!!!!!!!!!
			//osvrDeviceButtonConfigure(opts, &m_buttons, 1);

			/// Create the device token with the options
			m_dev.initAsync(ctx, PLUGIN_NAME, opts);

			/// Send JSON descriptor
			m_dev.sendJsonDescriptor(inf_psmove_osvr_connector_json);

			/// Register update callback
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
	PSMController *controllers[3] = {nullptr, nullptr, nullptr};
	int hmdControllerID = -1;
	int leftControllerID = -1;
	int rightControllerID = -1;
	
	struct AxisAngle { double a, x, y, z; };

	OSVR_PoseState PSMtoOSVRPoseState(const PSMPosef *psmPose) {

		// I have no idea what this does, i just managed to get it by trial and error. :)
		OSVR_PoseState pstate;
		osvrQuatSetIdentity(&(pstate.rotation));
		osvrVec3Zero(&(pstate.translation));

		PSMQuatf c_quat = psmPose->Orientation;
		/* Reverse Orientation
		osvrVec3SetX(&(pstate.translation), (-psmPose->Position.x / 100));
		osvrVec3SetY(&(pstate.translation), (psmPose->Position.y / 100));
		osvrVec3SetZ(&(pstate.translation), (-psmPose->Position.z / 100)); 
		boost::math::quaternion<double> psm_quat(-c_quat.w, c_quat.x, -c_quat.y, c_quat.z);
		*/

		osvrVec3SetX(&(pstate.translation), (psmPose->Position.x / 100));
		osvrVec3SetY(&(pstate.translation), (psmPose->Position.y / 100));
		osvrVec3SetZ(&(pstate.translation), (psmPose->Position.z / 100));
		boost::math::quaternion<double> psm_quat(c_quat.w, c_quat.x, c_quat.y, c_quat.z);

		osvrQuatSetW(&(pstate.rotation), psm_quat.R_component_4()); //w
		osvrQuatSetX(&(pstate.rotation), psm_quat.R_component_1()); //x
		osvrQuatSetY(&(pstate.rotation), psm_quat.R_component_2()); //y
		osvrQuatSetZ(&(pstate.rotation), psm_quat.R_component_3()); //z

		return pstate;
	}

	OSVR_Quaternion quatNormalize(OSVR_Quaternion &original) {
		double a, b, c, d;
		a = osvrQuatGetW(&original);
		b = osvrQuatGetX(&original);
		c = osvrQuatGetY(&original);
		d = osvrQuatGetZ(&original);
		double factor = std::sin(a / 2.0);
		// Calculate the x, y and z of the quaternion
		double x = b * factor;
		double y = c * factor;
		double z = d * factor;
		// Calculate the w value by cos( theta / 2 )
		double w = cos(a / 2.0);
		double len = std::sqrt(w*w + x*x + y*y + z*z);
		OSVR_Quaternion normalized = { w, x, y, z };
		return normalized;
	}

	AxisAngle quat2AAngle(OSVR_Quaternion &original) {
		OSVR_Quaternion inner = original;
		if (osvrQuatGetW(&inner) > 1) { // if w>1 acos and sqrt will produce errors, this cant happen if quaternion is normalised
			inner = quatNormalize(inner);
		}
		double angle = 2 * std::acosf(osvrQuatGetW(&inner));
		double s = std::sqrt(1 - osvrQuatGetW(&inner)*osvrQuatGetW(&inner)); // assuming quaternion normalised then w is less than 1, so term always positive.
		double x, y, z;
		if (s < 0.001) { // test to avoid divide by zero, s is always positive due to sqrt
						 // if s close to zero then direction of axis not important
			x = osvrQuatGetX(&inner); // if it is important that axis is normalised then replace with x=1; y=z=0;
			y = osvrQuatGetY(&inner);
			z = osvrQuatGetZ(&inner);
		}
		else {
			x = osvrQuatGetX(&inner) / s; // normalise axis
			y = osvrQuatGetY(&inner) / s;
			z = osvrQuatGetZ(&inner) / s;
		}
		return{ angle, x, y, z };
	}

	OSVR_Quaternion aAngle2Quat(double a1, double x1, double y1, double z1) {
		double w, x, y, z;
		x = x1 * std::sin(a1 / 2);
		y = y1 * std::sin(a1 / 2);
		z = z1 * std::sin(a1 / 2);
		w = std::cos(a1 / 2);
		OSVR_Quaternion newQuat;
		osvrQuatSetW(&newQuat, w);
		osvrQuatSetX(&newQuat, x);
		osvrQuatSetY(&newQuat, y);
		osvrQuatSetZ(&newQuat, z);

		return newQuat;
	}

};

class PSMS_OSVR_Constructor {
public:
	/// @brief This is the required signature for a device instantiation
	/// callback.
	OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx, const char *params) {

		
		// Connect to PSMS
		std::cout << "[" << PLUGIN_NAME << "]: Connecting to PSMoveService" << std::endl;

		// Connect to the PSM Server (0 - found psmove service, anything else error)
		int connectResult = PSM_Initialize("localhost", "9512", 1000);

		if(connectResult != 0) {
			std::cout << "[" << PLUGIN_NAME << "]: Could not connect to PSMoveService, make sure to start it before OSVR." << std::endl;
			return OSVR_RETURN_FAILURE;
		}

		// Read the JSON data from parameters.
		Json::Value root;
		if (params) {
			Json::Reader r;
			if (!r.parse(params, root)) {
				std::cout << "[" << PLUGIN_NAME << "]: Could not parse config parameters!" << std::endl;
				return OSVR_RETURN_FAILURE;
			}
		}

		std::cout << root << std::endl;

		int hmdControllerID = root.get("hmdController", -1).asInt();
		int leftHandControllerID = root.get("leftHandController", -1).asInt();
		int rightHandControllerID = root.get("rightHandController", -1).asInt();


		// OK, now that we have our parameters, create the device.
		osvr::pluginkit::registerObjectForDeletion(ctx, new PSMSDevice(ctx, hmdControllerID, leftHandControllerID, rightHandControllerID));

		return OSVR_RETURN_SUCCESS;
	}

private:
};

} // namespace

OSVR_PLUGIN(inf_psmove_osvr_connector) {

    osvr::pluginkit::PluginContext context(ctx);
	osvr::pluginkit::registerDriverInstantiationCallback(ctx, "PSMSDevice", new PSMS_OSVR_Constructor);

    return OSVR_RETURN_SUCCESS;
}