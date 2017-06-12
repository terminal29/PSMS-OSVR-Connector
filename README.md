# OSVR Move
A plugin for OSVR that add support for [PSMoveService](https://github.com/cboulay/PSMoveService/) controllers and HMD's.

# How to use 
Download the latest release, put *inf_osvr_move.dll* into your *osvr-plugins-0* folder, and put *PSMoveClient_CAPI.dll* in the same folder as your OSVR server executable.

Take a look at this example OSVR server config file.
```
  "drivers": [{
		"plugin": "inf_osvr_move",
		"driver": "MoveDevice",
		"params": {
			"params": {
				"debug":true,
				"controllers":[
					{
						"name":"regular_controller",
						"type":0,
						"id": 0
					},
					{
						"name":"pingPongHMD",
						"type":3,
						"id": 1
					},
					{
						"name":"NAVI CONTROLLER",
						"type":1,
						"id": 2
					}
				]
			}
		}
	}],
	"aliases": {
		"/me/head": "/inf_osvr_move/MoveDevice/semantic/pingPongHMD/tracker",
		"/me/hands/right": "/inf_osvr_move/MoveDevice/semantic/regular_controller/tracker",
		"/me/hands/right/button_0": "/inf_osvr_move/MoveDevice/semantic/NAVI CONTROLLER/cross"
	}
```
Each controller must have an entry in the "controllers" array for it to be linked. A controller must have a:
- "name" (Some string that identifies that controller. Used in the dynamic path generation)
- "type" (0 - Move Controller, 1 - Navi Controller, 2 - PSVR/Morpheus Headset, 3 - Virtual HMD)
- "id" (the id of the controller displayed in the PSMove Config Tool)

Once you put the controller in the "controllers" array, you will need to link it to some path. In the example the "regular_controller" is linked to my right hand tracker, "pingPongHMD" is linked to my head tracker, and I've linked the "cross" button from the "NAVI CONTROLLER" to my right hand "button_0". You can link anything to anything, it doesn't have to be in some specific order, but it is pointless linking a headset to some buttons, or a Navi Controller to a tracker because Headsets don't have buttons, and the Navi Controller has no tracking data.

The "debug" parameter, when switched to true, will print the dynamically generated paths for each controller so you can see their names and link them properly.

**notes**
- Make sure to start PSMoveService before OSVR Server
- Updated to support PSMoveService 0.9 alpha 8.4.0.
- Navi & HMD support is untested but *should* work.
