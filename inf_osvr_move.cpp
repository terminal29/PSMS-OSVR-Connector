
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>
#include <osvr/PluginKit/AnalogInterfaceC.h>
#include <osvr/PluginKit/ButtonInterfaceC.h>

#include <iostream>

#include <PSMoveClient_CAPI.h>
#include <ClientGeometry_CAPI.h>

#include <json/json.h>

namespace 
{
	const std::string DEVICE_NAME = "MoveDevice";
	std::vector<std::pair<std::string, PSMController*>> move_controllers;
	std::vector<std::pair<std::string, PSMController*>> navi_controllers;
	std::vector<std::pair<std::string, PSMController*>> ds4_controllers;
	std::vector<std::pair<std::string, PSMController*>> virtual_controllers;
	std::vector<std::pair<std::string, PSMHeadMountedDisplay*>> virtual_hmds;
	std::vector<std::pair<std::string, PSMHeadMountedDisplay*>> psvr_hmds;
	bool display_json = false;

	std::string psm_error_str(PSMResult result) {
		switch (result) {
		case PSMResult_Canceled:
			return "Connection cancelled";
		case PSMResult_Error:
			return "Connection error";
		case PSMResult_NoData:
			return "No data received";
		case PSMResult_Timeout:
			return "Connection timed out";
		default:
			return "Unknown error code";
		}
	}

	class Logger {
	public:
		Logger(OSVR_PluginRegContext& ctx):ctx_(ctx){};
		~Logger() {};
		std::ostream& get() {
			return log_stream_;
		}
		void send(bool flush = true) {
			OSVR_LogLevel level = OSVR_LOGLEVEL_INFO;
			if (warning_)
				level = OSVR_LOGLEVEL_WARN;
			osvr::pluginkit::log(ctx_, level, log_stream_.str().c_str());
			if (flush) {
				log_stream_.str("");
				warning_ = false;
			}
		}
		void set_warning(bool toggle) {
			warning_ = toggle;
		}
		private:
			OSVR_PluginRegContext& ctx_;
			std::stringstream log_stream_;
			bool warning_ = false;
	};

	class MoveDevice {
	public:

		MoveDevice(OSVR_PluginRegContext ctx){
			OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);
			Json::Value json_descriptor = generate_json_descriptor();
			if (display_json) {
				Logger log(ctx);
				log.get() << json_descriptor.toStyledString();
				log.send();
			}
			osvrDeviceTrackerConfigure(opts, &m_tracker);
			osvrDeviceButtonConfigure(opts, &m_buttons, json_descriptor["interfaces"]["button"]["count"].asInt());
			osvrDeviceAnalogConfigure(opts, &m_analog, json_descriptor["interfaces"]["analog"]["count"].asInt());
			m_dev.initAsync(ctx, DEVICE_NAME, opts);
			m_dev.sendJsonDescriptor(json_descriptor.toStyledString());
			m_dev.registerUpdateCallback(this);
		}

		OSVR_ReturnCode update() {
			PSM_UpdateNoPollMessages();

			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			
			int num_trackers = 0;
			int num_analogs = 0;
			int num_buttons = 0;

			// Update Move Controllers
			for (int i = 0; i < move_controllers.size(); i++) {
				std::string& con_name = move_controllers.at(i).first;
				PSMController* con = move_controllers.at(i).second;
				
				// Send Pose to Tracker
				PSMPosef con_pose_p = con->ControllerState.PSMoveState.Pose;
				OSVR_PoseState con_pose_o = psm_to_osvr_posestate(&con_pose_p);
				osvrDeviceTrackerSendPose(m_dev, m_tracker, &con_pose_o, num_trackers++);

				// Send analog (trigger) value
				osvrDeviceAnalogSetValue(m_dev, m_analog, con->ControllerState.PSMoveState.TriggerValue, num_analogs++);

				// Send button values
				PSMButtonState con_button_states[] = {
					con->ControllerState.PSMoveState.TriangleButton,
					con->ControllerState.PSMoveState.CircleButton,
					con->ControllerState.PSMoveState.CrossButton,
					con->ControllerState.PSMoveState.SquareButton,
					con->ControllerState.PSMoveState.SelectButton,
					con->ControllerState.PSMoveState.StartButton,
					con->ControllerState.PSMoveState.PSButton,
					con->ControllerState.PSMoveState.MoveButton,
					con->ControllerState.PSMoveState.TriggerButton
				};
				for (int j = 0; j < std::size(con_button_states); j++)
					osvrDeviceButtonSetValue(m_dev, m_buttons, (con_button_states[j] == PSMButtonState_DOWN), num_buttons++);

			}

			for (int i = 0; i < navi_controllers.size(); i++) {
				std::string& con_name = navi_controllers.at(i).first;
				PSMController* con = navi_controllers.at(i).second;

				// Send Analogs
				int con_analog_states[] = {
					con->ControllerState.PSNaviState.TriggerValue,
					con->ControllerState.PSNaviState.Stick_XAxis,
					con->ControllerState.PSNaviState.Stick_YAxis
				};
				for(int j = 0; j < std::size(con_analog_states); j++)
					osvrDeviceAnalogSetValue(m_dev, m_analog, con_analog_states[j], num_analogs++);

				// Send buttons
				PSMButtonState con_button_states[] = {
					con->ControllerState.PSNaviState.L1Button,
					con->ControllerState.PSNaviState.L2Button,
					con->ControllerState.PSNaviState.L3Button,
					con->ControllerState.PSNaviState.CircleButton,
					con->ControllerState.PSNaviState.CrossButton,
					con->ControllerState.PSNaviState.PSButton,
					con->ControllerState.PSNaviState.TriggerButton,
					con->ControllerState.PSNaviState.DPadUpButton,
					con->ControllerState.PSNaviState.DPadRightButton,
					con->ControllerState.PSNaviState.DPadDownButton,
					con->ControllerState.PSNaviState.DPadLeftButton
				};
				
				for (int j = 0; j < std::size(con_button_states); j++)
					osvrDeviceButtonSetValue(m_dev, m_buttons, (con_button_states[j] == PSMButtonState_DOWN), num_buttons++);
			}

			for (int i = 0; i < ds4_controllers.size(); i++) {
				std::string& con_name = ds4_controllers.at(i).first;
				PSMController* con = ds4_controllers.at(i).second;

				// Send Pose to Tracker
				PSMPosef con_pose_p = con->ControllerState.PSDS4State.Pose;
				OSVR_PoseState con_pose_o = psm_to_osvr_posestate(&con_pose_p);
				osvrDeviceTrackerSendPose(m_dev, m_tracker, &con_pose_o, num_trackers++);

				// Send Analogs
				int con_analog_states[] = {
					(int)(255 * (con->ControllerState.PSDS4State.LeftAnalogX + 1) / 2),
					(int)(255 * (con->ControllerState.PSDS4State.LeftAnalogY + 1) / 2),
					(int)(255 * (con->ControllerState.PSDS4State.RightAnalogY + 1) / 2),
					(int)(255 * (con->ControllerState.PSDS4State.RightAnalogY + 1) / 2),
					(int)(255 * con->ControllerState.PSDS4State.LeftTriggerValue),
					(int)(255 * con->ControllerState.PSDS4State.RightTriggerValue)
				};
				for (int j = 0; j < std::size(con_analog_states); j++)
					osvrDeviceAnalogSetValue(m_dev, m_analog, con_analog_states[j], num_analogs++);

				// Send Buttons
				PSMButtonState con_button_states[] = {
					con->ControllerState.PSDS4State.DPadUpButton,
					con->ControllerState.PSDS4State.DPadDownButton,
					con->ControllerState.PSDS4State.DPadLeftButton,
					con->ControllerState.PSDS4State.DPadRightButton,
					con->ControllerState.PSDS4State.SquareButton,
					con->ControllerState.PSDS4State.CrossButton,
					con->ControllerState.PSDS4State.CircleButton,
					con->ControllerState.PSDS4State.TriangleButton,
					con->ControllerState.PSDS4State.L1Button,
					con->ControllerState.PSDS4State.R1Button,
					con->ControllerState.PSDS4State.L2Button,
					con->ControllerState.PSDS4State.R2Button,
					con->ControllerState.PSDS4State.L3Button,
					con->ControllerState.PSDS4State.R3Button,
					con->ControllerState.PSDS4State.ShareButton,
					con->ControllerState.PSDS4State.OptionsButton,
					con->ControllerState.PSDS4State.PSButton,
					con->ControllerState.PSDS4State.TrackPadButton
				};
				for (int j = 0; j < std::size(con_button_states); j++)
					osvrDeviceButtonSetValue(m_dev, m_buttons, (con_button_states[j] == PSMButtonState_DOWN), num_buttons++);

			}

			for (int i = 0; i < virtual_controllers.size(); i++) {
				std::string& con_name = virtual_controllers.at(i).first;
				PSMController* con = virtual_controllers.at(i).second;

				// Send Pose to Tracker
				PSMPosef con_pose_p = con->ControllerState.VirtualController.Pose;
				OSVR_PoseState con_pose_o = psm_to_osvr_posestate(&con_pose_p);
				osvrDeviceTrackerSendPose(m_dev, m_tracker, &con_pose_o, num_trackers++);

			}

			for (int i = 0; i < virtual_hmds.size(); i++) {
				std::string& hmd_name = virtual_hmds.at(i).first;
				PSMHeadMountedDisplay* hmd = virtual_hmds.at(i).second;
				
				// Send Pose to Tracker
				PSMPosef con_pose_p = hmd->HmdState.VirtualHMDState.Pose;
				OSVR_PoseState con_pose_o = psm_to_osvr_posestate(&con_pose_p);
				osvrDeviceTrackerSendPose(m_dev, m_tracker, &con_pose_o, num_trackers++);

			}

			for (int i = 0; i < psvr_hmds.size(); i++) {
				std::string& hmd_name = psvr_hmds.at(i).first;
				PSMHeadMountedDisplay* hmd = psvr_hmds.at(i).second;

				// Send Pose to Tracker
				PSMPosef con_pose_p = hmd->HmdState.MorpheusState.Pose;
				OSVR_PoseState con_pose_o = psm_to_osvr_posestate(&con_pose_p);
				osvrDeviceTrackerSendPose(m_dev, m_tracker, &con_pose_o, num_trackers++);

			}




			return OSVR_RETURN_SUCCESS;
		}

		~MoveDevice(){

		}

	private:
		osvr::pluginkit::DeviceToken m_dev;
		OSVR_TrackerDeviceInterface m_tracker;
		OSVR_ButtonDeviceInterface m_buttons;
		OSVR_AnalogDeviceInterface m_analog;


		Json::Value generate_json_descriptor() {
			Json::Value descriptor;
			descriptor["deviceVendor"] = "Sony";
			descriptor["deviceName"] = DEVICE_NAME;
			descriptor["author"] = "InfiniteLlamas";
			descriptor["version"] = "0.4a";
			//descriptor["lastModified"] = std::string(__DATE__).append("T").append(__TIME__).append("Z");

			Json::Value interfaces;
			Json::Value semantic;

			Json::Value tracker;
			tracker["position"] = true;
			tracker["orientation"] = true;
			interfaces["tracker"] = tracker;

			int num_trackers = 0;
			int num_analogs = 0;
			int num_buttons = 0;

			// Move Controllers
			for (int i = 0; i < move_controllers.size(); i++) {
				
				// Add tracker to semantic
				semantic[move_controllers.at(i).first + "/tracker"] = "tracker/" + std::to_string(num_trackers++);
				
				// Add buttons to semantic
				std::string button_names[] = { "/triangle","/circle","/cross","/square","/select","/start","/ps","/move","/triggerbtn" };
				for (int j = 0; j < std::size(button_names); j++) {
					semantic[move_controllers.at(i).first + button_names[j]] = "button/" + std::to_string(num_buttons++);
				}

				// Add analog (trigger) to semantic
				semantic[move_controllers.at(i).first + "/trigger"] = "analog/" + std::to_string(num_analogs++);
				
			}

			// Navigation Controllers
			for (int i = 0; i < navi_controllers.size(); i++) {
				
				//Add buttons to semantic
				std::string button_names[] = { "/l1","/l2","/l3","/circle","/cross","/ps","/triggerbtn","/dpadup","/dpadright","/dpaddown","/dpadleft" };
				for (int j = 0; j < std::size(button_names); j++)
					semantic[navi_controllers.at(i).first + button_names[j]] = "button/" + std::to_string(num_buttons++);
				
				//Add analogs to semantic
				std::string analog_names[] = { "/trigger", "/stickx","/sticky" };
				for (int j = 0; j < std::size(analog_names); j++)
					semantic[navi_controllers.at(i).first + analog_names[j]] = "analog/" + std::to_string(num_analogs++);
			}

			// Ds4 Controllers
			for (int i = 0; i < ds4_controllers.size(); i++) {

				// Add tracker to semantic
				semantic[ds4_controllers.at(i).first + "/tracker"] = "tracker/" + std::to_string(num_trackers++);

				//Add buttons to semantic
				std::string button_names[] = { "/dpadup","/dpaddown","/dpadleft","/dpadright", "/square","/cross","/circle","/triangle","/l1","/r1","/l2","/r2","/l3","/r3","/share","/options","/ps","/trackpad" };
				for (int j = 0; j < std::size(button_names); j++)
					semantic[ds4_controllers.at(i).first + button_names[j]] = "button/" + std::to_string(num_buttons++);

				//Add analogs to semantic
				std::string analog_names[] = { "/lstickx", "/lsticky", "rstickx", "rsticky", "/ltrigger", "/rtrigger"};
				for (int j = 0; j < std::size(analog_names); j++)
					semantic[ds4_controllers.at(i).first + analog_names[j]] = "analog/" + std::to_string(num_analogs++);


			}

			// Virtual Controllers
			for (int i = 0; i < virtual_controllers.size(); i++) {
				// Add tracker to semantic
				semantic[virtual_controllers.at(i).first + "/tracker"] = "tracker/" + std::to_string(num_trackers++);

				// TODO: Do virtual controllers have buttons & analog inputs ?
			}

			// Virtual HMDs
			for (int i = 0; i < virtual_hmds.size(); i++) {
				// Add tracker to semantic
				semantic[virtual_hmds.at(i).first + "/tracker"] = "tracker/" + std::to_string(num_trackers++);

			}

			// PSVR HMDs
			for (int i = 0; i < psvr_hmds.size(); i++) {
				// Add tracker to semantic
				semantic[psvr_hmds.at(i).first + "/tracker"] = "tracker/" + std::to_string(num_trackers++);

			}

			Json::Value analog;
			analog["count"] = num_analogs;
			Json::Value traits(Json::arrayValue);
			Json::Value trait_value;
			trait_value["min"] = 0;
			trait_value["max"] = 255;
			traits[0] = trait_value;
			analog["traits"] = traits;

			Json::Value button;
			button["count"] = num_buttons;

			
			interfaces["analog"] = analog;
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
			Logger log(ctx);

			log.get() << "Attempting connection with PSMoveService...";
			log.send();

			// Attempt to connect to the PSMoveService server
			for (int i = 0; i < 5; i++) {
				
				log.get() << "Attempt " << (i+1);
				log.send();
				
				PSMResult result = PSM_Initialize(PSMOVESERVICE_DEFAULT_ADDRESS, PSMOVESERVICE_DEFAULT_PORT, PSM_DEFAULT_TIMEOUT);
				
				if (result == PSMResult_Success)
					break;

				if (i == 4) {
					log.get() << "Failed to connect to PSMoveService with error: [" << result << "] " << psm_error_str(result);
					log.set_warning(true);
					log.send();
					return OSVR_RETURN_FAILURE;
				}
			}

			// Returns array index value appears at, -1 if it doesnt appear in the array
			auto find = [](int* arr, int count, int value) -> int {
				for (int i = 0; i < count; i++) {
					if (arr[i] == value)
						return i;
				}
				return -1;
			};

			// Attempt to connect all requested controllers
			Json::Value config_params;
			log.get() << "Attempting connection with controllers/HMDs...";
			log.send();
			if (params) {
				Json::Reader reader;
				bool parse_result = reader.parse(params, config_params);
				if (!parse_result) {
					log.get() << "Error occurred parsing config file:" << std::endl << reader.getFormattedErrorMessages();
					log.set_warning(true);
					log.send();
					return OSVR_RETURN_FAILURE;
				}
				try {
					display_json = config_params.get("debug", false).asBool();
					PSMHmdList hmd_list;
					PSMControllerList con_list;
					PSM_GetHmdList(&hmd_list, 5000);
					PSM_GetControllerList(&con_list, 5000);

					for (Json::Value controller : config_params["controllers"]) {

						std::string controller_name = controller["name"].asString();
						std::string controller_type = controller["type"].asString();
						int controller_id = controller.get("id", -1).asInt();

						log.get() << "Parsing device " << controller_name << " as a " << controller_type;
						log.send();

						int con_pos = find(con_list.controller_id, con_list.count, controller_id);
						int hmd_pos = find(hmd_list.hmd_id, hmd_list.count, controller_id);

						if (con_pos == -1 && hmd_pos == -1) {
							log.get() << "Controller or HMD [id:" << controller_id << "] \"" << controller_name << "\" is not connected, please connect the controller or HMD and restart OSVR.";
							log.set_warning(true);
							log.send();
							return OSVR_RETURN_FAILURE;
						}
						if (controller_type.compare("Move") == 0) { // We are looking for a move controller
							if (con_list.controller_type[con_pos] != PSMController_Move) {
								log.get() << "Controller [id:" << controller_id << "] \"" << controller_name << "\" is not a Move controller, please correct this and restart OSVR.";
								log.set_warning(true);
								log.send();
								return OSVR_RETURN_FAILURE;
							}
							PSMController* move_controller = PSM_GetController(controller_id);
							PSM_AllocateControllerListener(controller_id);
							PSM_StartControllerDataStream(controller_id, PSMStreamFlags_includePositionData | PSMStreamFlags_includeCalibratedSensorData | PSMStreamFlags_includePhysicsData, 5000);
							move_controllers.emplace_back(std::make_pair(controller_name, move_controller));
						}
						else if (controller_type.compare("Navi") == 0) { // We are looking for a navigation controller
							if (con_list.controller_type[con_pos] != PSMController_Navi) {
								log.get() << "Controller [id:" << controller_id << "] \"" << controller_name << "\" is not a Navigation controller, please correct this and restart OSVR.";
								log.set_warning(true);
								log.send();
								return OSVR_RETURN_FAILURE;
							}
							PSMController* navi_controller = PSM_GetController(controller_id);
							PSM_AllocateControllerListener(controller_id);
							PSM_StartControllerDataStream(controller_id, PSMStreamFlags_includePositionData | PSMStreamFlags_includeCalibratedSensorData | PSMStreamFlags_includePhysicsData, 5000);
							navi_controllers.emplace_back(std::make_pair(controller_name, navi_controller));
						}
						else if (controller_type.compare("DualShock4") == 0) { // We are looking for a ds4 controller
							if (con_list.controller_type[con_pos] != PSMController_DualShock4) {
								log.get() << "Controller [id:" << controller_id << "] \"" << controller_name << "\" is not a DualShock4 controller, please correct this and restart OSVR.";
								log.set_warning(true);
								log.send();
								return OSVR_RETURN_FAILURE;
							}
							PSMController* ds4_controller = PSM_GetController(controller_id);
							PSM_AllocateControllerListener(controller_id);
							PSM_StartControllerDataStream(controller_id, PSMStreamFlags_includePositionData | PSMStreamFlags_includeCalibratedSensorData | PSMStreamFlags_includePhysicsData, 5000);
							ds4_controllers.emplace_back(std::make_pair(controller_name, ds4_controller));
						}
						else if (controller_type.compare("VirtualMove") == 0) { // We are looking for a virtual controller
							if (con_list.controller_type[con_pos] != PSMController_Virtual) {
								log.get() << "Controller [id:" << controller_id << "] \"" << controller_name << "\" is not a VirtualMove controller, please correct this and restart OSVR.";
								log.set_warning(true);
								log.send();
								return OSVR_RETURN_FAILURE;
							}
							PSMController* virtual_controller = PSM_GetController(controller_id);
							PSM_AllocateControllerListener(controller_id);
							PSM_StartControllerDataStream(controller_id, PSMStreamFlags_includePositionData | PSMStreamFlags_includeCalibratedSensorData | PSMStreamFlags_includePhysicsData, 5000);
							virtual_controllers.emplace_back(std::make_pair(controller_name, virtual_controller));
						}
						else if (controller_type.compare("VirtualHMD") == 0) { // We are looking for a virtual HMD
							if (hmd_list.hmd_type[hmd_pos] != PSMHmd_Virtual) {
								log.get() << "HMD [id:" << controller_id << "] \"" << controller_name << "\" is not a VirtualHMD HMD, please correct this and restart OSVR.";
								log.set_warning(true);
								log.send();
								return OSVR_RETURN_FAILURE;
							}
							PSMHeadMountedDisplay* virtual_hmd = PSM_GetHmd(controller_id);
							PSM_AllocateHmdListener(controller_id);
							PSM_StartHmdDataStream(controller_id, PSMStreamFlags_includePositionData | PSMStreamFlags_includeCalibratedSensorData | PSMStreamFlags_includePhysicsData, 5000);
							virtual_hmds.emplace_back(std::make_pair(controller_name, virtual_hmd));

						}
						else if (controller_type.compare("PSVR") == 0) { // We are looking for a Morpheus headset
							if (hmd_list.hmd_type[hmd_pos] != PSMHmd_Virtual) {
								log.get() << "HMD [id:" << controller_id << "] \"" << controller_name << "\" is not a PSVR HMD, please correct this and restart OSVR.";
								log.set_warning(true);
								log.send();
								return OSVR_RETURN_FAILURE;
							}
							PSMHeadMountedDisplay* psvr_hmd = PSM_GetHmd(controller_id);
							PSM_AllocateHmdListener(controller_id);
							PSM_StartHmdDataStream(controller_id, PSMStreamFlags_includePositionData | PSMStreamFlags_includeCalibratedSensorData | PSMStreamFlags_includePhysicsData, 5000);
							psvr_hmds.emplace_back(std::make_pair(controller_name, psvr_hmd));
						}
						else {
							log.get() << "Unknown controller/HMD type \"" << controller_type << "\" for controller id " << controller_id;
							log.set_warning(true);
							log.send();
							log.get() << "Valid types are Move, Navi, DualShock4, VirtualMove, VirtualHMD, and PSVR." ;
							log.set_warning(true);
							log.send();
							return OSVR_RETURN_FAILURE;
						}


					}
				}
				catch (Json::Exception exc) {
					log.get() << "Exception occured while loading config:" << std::endl << exc.what();
					log.set_warning(true);
					log.send();
					return OSVR_RETURN_FAILURE;
				}
			}
			else {
				log.get() << "No controllers specified...";
				log.set_warning(true);
				log.send();
				return OSVR_RETURN_FAILURE;
			}

			log.get() << "Parsed all controllers/HMDs successfully.";
			log.send();

			osvr::pluginkit::registerObjectForDeletion(ctx, new MoveDevice(ctx));
			return OSVR_RETURN_SUCCESS;
		}
	};

} // namespace

OSVR_PLUGIN(inf_osvr_move) {
	osvr::pluginkit::PluginContext context(ctx);
	osvr::pluginkit::registerDriverInstantiationCallback(ctx, DEVICE_NAME.c_str(), new OSVR_Move_Constructor);
	return OSVR_RETURN_SUCCESS;
}

