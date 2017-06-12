#include "controller.h"

std::pair<int, std::string> device_types[5] = {
	{ 0,"PS Move Controller" },
	{ 1,"PS Navi Controller" },
	{ 2,"PSVR Headset" },
	{ 3,"Virtual HMD" },
	{ 4,"Null Device" }
};

Controller::Controller(std::string name, int device_id, int device_type):device_name(name),device_id(device_id),device_type(device_type){
}

Controller::~Controller(){
}
