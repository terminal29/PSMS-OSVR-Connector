# PSMS-OSVR-Connector
Plugin for OSVR that uses PSMoveService to connect HMD, left, and right hands to PS Move controllers.

# How to use 
Just extract the plugin dll and json to your OSVR server plugins folder, and from your PSMoveService folder you should find a PSMoveService_CAPI.dll file, copy that to your OSVR server root/bin folder. Please ensure it is at least from version >= PSMS v9.0a8.2.0.

To specify which controller should be used for which tracker, include this in your osvr server config:
```
  "drivers": [{
		"plugin": "inf_psmove_osvr_connector",
		"driver": "PSMSDevice",
		"params": {
			"hmdController": 0,
			"leftHandController": 1,
			"rightHandController": 2
		}
	}],
	"aliases": {
		"/me/head": "/inf_psmove_osvr_connector/PSMS OSVR Plugin/semantic/hmd",
		"/me/hands/left": "/inf_psmove_osvr_connector/PSMS OSVR Plugin/semantic/left",
		"/me/hands/right": "/inf_psmove_osvr_connector/PSMS OSVR Plugin/semantic/right"
	}
```

Where 0, 1, and 2 are the *controller ID* in PSMoveConfig of the controllers you wish to map.

If you dont want to map the left and right hands, and just use the head, remove them from the params.

It will make a tracker device regardless of whether a controller actually exists there, and will not reconnect to a controller if it goes flat for example, I will be working on fixing this some time soon but for now it works enough.

There are no configs for rotation offsets etc. 

TODO (priority in order):
- Button support
- Offsets
- Auto-Reconnect to PSMS
- Handle properly if controller doesn't exist
- Clean up stdout
- eta son.

**notes**
- Start PSMS before OSVR Server
- Tested with PSMS alpha ver. 8.2.0
- Works with OSVR-Steamvr for HMD Only, I don't think OSVR-Steamvr works with OSVR defined controllers just yet.
