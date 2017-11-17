# OSVR Move
(formerly PSMS-OVSR-Connector)
A plugin for OSVR that add support for [PSMoveService](https://github.com/cboulay/PSMoveService/) controllers and HMD's.

# How to use 
Download the latest release, put *inf_osvr_move.dll* into your *osvr-plugins-0* folder, and put *PSMoveClient_CAPI.dll* in the same folder as your OSVR server executable.

Take a look at this example OSVR server config file snippit.
```json
  "drivers": [{
		"plugin": "inf_osvr_move",
		"driver": "MoveDevice",
		"params": {
			"debug":true,
			"controllers":[
				{
					"name":"controller1",
					"type":"Move",
					"id": 0
				},
				{
					"name":"controller2",
					"type":"Move",
					"id": 1
				}
				,
				{
					"name":"navi1",
					"type":"Navi",
					"id": 2
				},
				{
					"name":"navi2",
					"type":"Navi",
					"id": 3
				}
				,
				{
					"name":"hmd",
					"type":"VirtualHMD",
					"id": 0
				}
			]
		}
	}],
	"aliases": {
		"/me/head": "/inf_osvr_move/MoveDevice/semantic/hmd/tracker",
		"/me/hands/left": "/inf_osvr_move/MoveDevice/semantic/controller1/tracker",
		"/me/hands/right": "/inf_osvr_move/MoveDevice/semantic/controller2/tracker"
	}
```
Each controller and HMD must have an entry in the "controllers" array for it to be linked. A device must have a:
- "name" (Some string that identifies that controller or HMD. Used in the dynamic path generation)
- "type" (Move, Navi, DualShock4, VirtualMove, VirtualHMD, or PSVR)
- "id" (the id of the device as displayed in the PSMove Config Tool)

Once you put the device in the "controllers" array, you will need to link it to some path. In the example the "controller1" is linked to my left hand tracker, "controller2" is linked to my right hand tracker, and "hmd" is a virtual HMD linked to my head position.

The "debug" parameter, when switched to true, will print the dynamically generated paths for each controller so you can see their names and link them properly.

**notes**
- Make sure to start PSMoveService before OSVR Server
- Updated to support PSMoveService 0.9 alpha 8.8.0.
- DS4 and PSVR HMD support is included but untested.
