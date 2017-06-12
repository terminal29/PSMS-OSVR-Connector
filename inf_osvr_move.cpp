// Internal Includes
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>
#include <osvr/PluginKit/AnalogInterfaceC.h>
#include <osvr/PluginKit/ButtonInterfaceC.h>

// Generated JSON header file
#include "inf_osvr_move.h"

// Standard includes
#include <chrono>
#include <thread>
#include <iostream>
#include <string>
#include <json/json.h>

// 3rd party includes
#include <PSMoveClient_CAPI.h>
#include <ClientGeometry_CAPI.h>

// 1st party includes
#include "controller.h"
#include "util.h"

#define DEFAULT_TIMEOUT 5000
#define DEVICE_NAME "MoveDevice"

namespace {
	class Move_Device {
		public:

		Move_Device(OSVR_PluginRegContext ctx, std::vector<Controller> req_controllers, bool display_JSON):controllers(req_controllers) {
			OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);
			Json::Value json_descriptor = generate_json_descriptor();
			osvrDeviceTrackerConfigure(opts, &m_tracker);
			osvrDeviceButtonConfigure(opts, &m_buttons, json_descriptor["interfaces"]["button"]["count"].asInt());
			osvrDeviceAnalogConfigure(opts, &m_analog, json_descriptor["interfaces"]["analog"]["count"].asInt());
			m_dev.initAsync(ctx, DEVICE_NAME, opts);
			if (display_JSON)
				std::cout << PRNT_PFX << json_descriptor.toStyledString() << std::endl;
			m_dev.sendJsonDescriptor(json_descriptor.toStyledString());
			m_dev.registerUpdateCallback(this);
		}

		OSVR_ReturnCode update() {
			PSM_UpdateNoPollMessages();

			std::this_thread::sleep_for(std::chrono::milliseconds(10));

			//Check for connections and validity

			int num_tracker_values = 0;
			int num_analog_values = 0;
			int num_digital_buttons = 0;
			for (int i = 0; i < controllers.size(); i++) {
				if (controllers.at(i).device_type == 0) {
					PSMController* remote_con_ptr = (PSMController*)(controllers.at(i).psm_device_ptr);
					PSMPosef con_pose = remote_con_ptr->ControllerState.PSMoveState.Pose;
					PSMButtonState con_button_states[9] = {
						remote_con_ptr->ControllerState.PSMoveState.TriangleButton,
						remote_con_ptr->ControllerState.PSMoveState.CircleButton,
						remote_con_ptr->ControllerState.PSMoveState.CrossButton,
						remote_con_ptr->ControllerState.PSMoveState.SquareButton,
						remote_con_ptr->ControllerState.PSMoveState.SelectButton,
						remote_con_ptr->ControllerState.PSMoveState.StartButton,
						remote_con_ptr->ControllerState.PSMoveState.PSButton,
						remote_con_ptr->ControllerState.PSMoveState.MoveButton,
						remote_con_ptr->ControllerState.PSMoveState.TriggerButton
					};

					int con_analog_state = remote_con_ptr->ControllerState.PSMoveState.TriggerValue;

					osvrDeviceAnalogSetValue(m_dev, m_analog, con_analog_state, num_analog_values);
					num_analog_values++;
					
					OSVR_PoseState osvr_con_pose = psm_to_osvr_posestate(&con_pose);
					osvrDeviceTrackerSendPose(m_dev, m_tracker, &osvr_con_pose, num_tracker_values);
					num_tracker_values++;

					for (int j = 0; j < 9; j++) {
						osvrDeviceButtonSetValue(m_dev, m_buttons, (con_button_states[j] == PSMButtonState_DOWN), num_digital_buttons);
						num_digital_buttons++;
					}
				}
				if (controllers.at(i).device_type == 1) {
					PSMController* remote_con_ptr = (PSMController*)(controllers.at(i).psm_device_ptr);
					PSMButtonState con_button_states[11] = {
						remote_con_ptr->ControllerState.PSNaviState.L1Button,
						remote_con_ptr->ControllerState.PSNaviState.L2Button,
						remote_con_ptr->ControllerState.PSNaviState.L3Button,
						remote_con_ptr->ControllerState.PSNaviState.CircleButton,
						remote_con_ptr->ControllerState.PSNaviState.CrossButton,
						remote_con_ptr->ControllerState.PSNaviState.PSButton,
						remote_con_ptr->ControllerState.PSNaviState.TriggerButton,
						remote_con_ptr->ControllerState.PSNaviState.DPadUpButton,
						remote_con_ptr->ControllerState.PSNaviState.DPadRightButton,
						remote_con_ptr->ControllerState.PSNaviState.DPadDownButton,
						remote_con_ptr->ControllerState.PSNaviState.DPadLeftButton
					};
					int con_analog_states[3] = {
						remote_con_ptr->ControllerState.PSNaviState.TriggerValue,
						(int)(remote_con_ptr->ControllerState.PSNaviState.Stick_XAxis * 255),
						(int)(remote_con_ptr->ControllerState.PSNaviState.Stick_YAxis * 255)
					};
					for (int j = 0; j < 3; j++) {
						osvrDeviceAnalogSetValue(m_dev, m_analog, con_analog_states[j], num_analog_values);
						num_analog_values++;
					}
					for (int j = 0; j < 11; j++) {
						osvrDeviceButtonSetValue(m_dev, m_buttons, (con_button_states[j] == PSMButtonState_DOWN), num_digital_buttons);
						num_digital_buttons++;
					}
				}
				if (controllers.at(i).device_type == 2) {
					PSMHeadMountedDisplay* remote_hdm_ptr = (PSMHeadMountedDisplay*)(controllers.at(i).psm_device_ptr);
					PSMPosef hmd_pose = remote_hdm_ptr->HmdState.MorpheusState.Pose;
					OSVR_PoseState osvr_hmd_pose = psm_to_osvr_posestate(&hmd_pose);
					osvrDeviceTrackerSendPose(m_dev, m_tracker, &osvr_hmd_pose, num_tracker_values);
					num_tracker_values++;
				}
				if (controllers.at(i).device_type == 3) {
					PSMHeadMountedDisplay* remote_hdm_ptr = (PSMHeadMountedDisplay*)(controllers.at(i).psm_device_ptr);
					PSMPosef hmd_pose = remote_hdm_ptr->HmdState.VirtualHMDState.Pose;
					OSVR_PoseState osvr_hmd_pose = psm_to_osvr_posestate(&hmd_pose);
					osvrDeviceTrackerSendPose(m_dev, m_tracker, &osvr_hmd_pose, num_tracker_values);
					num_tracker_values++;
				}
			}
			return OSVR_RETURN_SUCCESS;
		}

	private:
		std::vector<Controller> controllers;
		osvr::pluginkit::DeviceToken m_dev;
		OSVR_TrackerDeviceInterface m_tracker;
		OSVR_ButtonDeviceInterface m_buttons;
		OSVR_AnalogDeviceInterface m_analog;

		Json::Value generate_json_descriptor() {
			Json::Value descriptor;
			descriptor["deviceVendor"] = "Sony";
			descriptor["deviceName"] = DEVICE_NAME;
			descriptor["author"] = "InfiniteLlamas";
			descriptor["version"] = "0.3a";
			//descriptor["lastModified"] = std::string(__DATE__).append("T").append(__TIME__).append("Z");
			
			Json::Value interfaces;
			Json::Value semantic;
			
			Json::Value tracker;
			tracker["position"] = true;
			tracker["orientation"] = true;
			int num_tracker_values = 0;
			for (int i = 0; i < controllers.size(); i++) {
				if (controllers.at(i).device_type == 0 || controllers.at(i).device_type == 2 || controllers.at(i).device_type == 3) {
					
					std::stringstream tracker_desc;
					tracker_desc << "tracker/" << num_tracker_values;
					std::string dev_name_tracker = controllers.at(i).device_name;
					semantic[dev_name_tracker.append("/tracker")] = tracker_desc.str();
					
					num_tracker_values++;
					
				}
			}
			interfaces["tracker"] = tracker;

			Json::Value analog;
			int num_analog_values = 0;
			for (int i = 0; i < controllers.size(); i++) {
				if (controllers.at(i).device_type == 0) {

					std::stringstream tracker_desc;
					tracker_desc << "analog/" << num_analog_values;
					std::string dev_name = controllers.at(i).device_name;
					semantic[dev_name.append("/trigger")] = tracker_desc.str();

					num_analog_values++;
				}
				if (controllers.at(i).device_type == 1) {

					std::stringstream tracker_desc;
					std::string dev_name = controllers.at(i).device_name;
					tracker_desc << "analog/" << num_analog_values;
					semantic[dev_name.append("/trigger")] = tracker_desc.str();
					tracker_desc.str("");
					tracker_desc << "analog/" << num_analog_values + 1;
					dev_name = controllers.at(i).device_name;
					semantic[dev_name.append("/stickx")] = tracker_desc.str();
					tracker_desc.str("");
					tracker_desc << "analog/" << num_analog_values + 2;
					dev_name = controllers.at(i).device_name;
					semantic[dev_name.append("/sticky")] = tracker_desc.str();

					num_analog_values += 3;
				}
			}
			analog["count"] = num_analog_values;
			Json::Value traits(Json::arrayValue);
			Json::Value trait_value;
			trait_value["min"] = 0;
			trait_value["max"] = 255;
			traits[0] = trait_value;
			analog["traits"] = traits;
			interfaces["analog"] = analog;

			Json::Value button;
			int num_digital_buttons = 0;
			for (int i = 0; i < controllers.size(); i++) {
				if (controllers.at(i).device_type == 0) {
					std::string button_names[9] = { "/triangle","/circle","/cross","/square","/select","/start","/ps","/move","/triggerbtn" };
					std::stringstream button_desc;
					std::string dev_name;
					for (int j = 0; j < 9; j++) {
						button_desc.str("");
						button_desc << "button/" << num_digital_buttons;
						dev_name = controllers.at(i).device_name;
						semantic[dev_name.append(button_names[j])] = button_desc.str();
						num_digital_buttons++;
					}
				}
				if (controllers.at(i).device_type == 1) {
					std::string button_names[11] = { "/l1","/l2","/l3","/circle","/cross","/ps","/triggerbtn","/dpadup","/dpadright","/dpaddown","/dpadleft" };
					std::stringstream button_desc;
					std::string dev_name;
					for (int j = 0; j < 11; j++) {
						button_desc.str("");
						button_desc << "button/" << num_digital_buttons;
						dev_name = controllers.at(i).device_name;
						semantic[dev_name.append(button_names[j])] = button_desc.str();
						num_digital_buttons++;
					}
				}
			}
			button["count"] = num_digital_buttons;
			interfaces["button"] = button;
			descriptor["interfaces"] = interfaces;
			descriptor["semantic"] = semantic;

			return descriptor;
		}

		OSVR_PoseState psm_to_osvr_posestate(const PSMPosef *psm_pose) {
			OSVR_PoseState pstate;
			osvrQuatSetIdentity(&(pstate.rotation));
			osvrVec3Zero(&(pstate.translation));

			// To convert PSMS Meters into OSVR Centimeters
			osvrVec3SetX(&(pstate.translation), (psm_pose->Position.x / 100.0));
			osvrVec3SetY(&(pstate.translation), (psm_pose->Position.y / 100.0));
			osvrVec3SetZ(&(pstate.translation), (psm_pose->Position.z / 100.0));

			// Just pass straight through for now
			osvrQuatSetW(&(pstate.rotation), psm_pose->Orientation.w); //w
			osvrQuatSetX(&(pstate.rotation), psm_pose->Orientation.x); //x
			osvrQuatSetY(&(pstate.rotation), psm_pose->Orientation.y); //y
			osvrQuatSetZ(&(pstate.rotation), psm_pose->Orientation.z); //z

			return pstate;
		}
	};

	class OSVR_Move_Constructor {
	public:
		OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx, const char *params) {
			std::vector<Controller> controllers;
			bool display_JSON = false;
			std::cout << PRNT_PFX << " Loading config..." << std::endl;
			Json::Value config_params;
			if (params) {
				Json::Reader reader;
				bool parse_result = reader.parse(params, config_params);
				if (!parse_result) {
					std::cout << PRNT_PFX << " Invalid config, load failed!" << std::endl;
					return OSVR_RETURN_FAILURE;
				}
				display_JSON = config_params.get("debug", false).asBool();
				int req_con_num = 0;
				for (Json::Value controller : config_params["controllers"]) {
					req_con_num++;
					std::string cur_con_name = controller.get("name", "invalid name").asString();
					int cur_con_id = controller.get("id", -1).asInt();
					int cur_con_type = controller.get("type", -1).asInt();
					if (cur_con_name.length() > 0 && cur_con_id >= 0 && 5 > cur_con_type && cur_con_type >= 0) {
						Controller cur_controller = Controller(cur_con_name, cur_con_id, cur_con_type);
						controllers.push_back(cur_controller);
						std::cout << PRNT_PFX << " Parsed controller #" << cur_con_id << " " << cur_con_name << " as a " << (device_types)[cur_con_type].second << std::endl;
					}
					else {
						std::cout << PRNT_PFX << " Unable to parse controller #" << req_con_num << std::endl;
					}
				}
			}
			else {
				return OSVR_RETURN_SUCCESS;
			}
			
			std::cout << PRNT_PFX << " Attempting connection to PSMove Service..." << std::endl;
			PSMResult connect_result;
			int connect_attempts = 0;
			do {
				std::cout << PRNT_PFX << " Attempt #" << connect_attempts + 1 << std::endl;
				connect_result = PSM_Initialize(PSMOVESERVICE_DEFAULT_ADDRESS, PSMOVESERVICE_DEFAULT_PORT, DEFAULT_TIMEOUT);
				connect_attempts++;
			} while (connect_result != PSMResult_Success && connect_attempts < 5);

			if (connect_result != PSMResult_Success) {
				std::cout << PRNT_PFX << " Connection failed: [" << connect_result << "] " << util::get_psm_error_str(connect_result) << std::endl;
				return OSVR_RETURN_FAILURE;
			}

			std::cout << PRNT_PFX << " Linking controllers..." << std::endl;
			for (int i = 0; i < controllers.size(); i++) {
				init_controller(&(controllers.at(i)));
			}
			
			osvr::pluginkit::registerObjectForDeletion(ctx, new Move_Device(ctx, controllers, display_JSON));
			return OSVR_RETURN_SUCCESS;
		}
	private:
		void init_controller(Controller* con) {
			if (con->device_type == 0 || con->device_type == 1) {
				PSMController* remote_controller = PSM_GetController(con->device_id);
				PSM_AllocateControllerListener(con->device_id);
				PSM_StartControllerDataStream(con->device_id, PSMStreamFlags_includePositionData | PSMStreamFlags_includeCalibratedSensorData | PSMStreamFlags_includePhysicsData, DEFAULT_TIMEOUT);
				con->psm_device_ptr = remote_controller;
				con->is_valid = true;
			}
			else if (con->device_type == 2 || con->device_type == 3) {
				PSMHeadMountedDisplay* remote_hmd = PSM_GetHmd(con->device_id);
				PSM_AllocateHmdListener(con->device_id);
				PSM_StartHmdDataStream(con->device_id, PSMStreamFlags_includePositionData | PSMStreamFlags_includeCalibratedSensorData | PSMStreamFlags_includePhysicsData, DEFAULT_TIMEOUT);
				con->psm_device_ptr = remote_hmd;
				con->is_valid = true;
			}
		}
	};

} // namespace

OSVR_PLUGIN(osvr_move) {
	osvr::pluginkit::PluginContext context(ctx);
	osvr::pluginkit::registerDriverInstantiationCallback(ctx, DEVICE_NAME, new OSVR_Move_Constructor);
    return OSVR_RETURN_SUCCESS;
}