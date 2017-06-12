#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_
/* 
		Simple abstraction layer on top of the PSMove Service CAPI
*/

#include <PSMoveClient_CAPI.h>
#include <string>
#include <utility>

extern std::pair<int, std::string> device_types[5];

class Controller{
public:
	Controller(std::string name, int device_id, int device_type);
	~Controller();

	std::string device_name;
	float position[3] = { 0,0,0 };
	float q_rotation[4] = { 0,0,0,1 };
	int device_id = 0;
	int device_type = -1;
	bool is_valid = false;
	void* psm_device_ptr = nullptr;
	bool is_connected = false;
};


#endif
