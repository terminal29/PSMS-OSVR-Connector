# PSMS-OSVR-Connector
Plugin for OSVR that uses PSMoveService to connect HMD, left, and right hands to PS Move controllers.


# How to use 
- Download the plugin dll to your osvr-plugins-0 folder
- Copy PSMoveService_CAPI.dll (from your PSMS folder) to your OSVR server root folder (next to osvr_server.exe)
- Follow below to specify which controller maps to which tracker.

To specify which controller should be used for which tracker, include this in your osvr server config:
[Example server config](https://pastebin.com/UAW7pmsh)
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
		"/me/head": "/inf_psmove_osvr_connector/PSMSDevice/semantic/hmd",
		"/me/hands/left": "/inf_psmove_osvr_connector/PSMSDevice/semantic/left",
		"/me/hands/right": "/inf_psmove_osvr_connector/PSMSDevice/semantic/right"
	}
```

Where 0, 1, and 2 are the *controller ID* in PSMoveConfig of the controllers you wish to map.

To remove a controller from being tracked, simply remove it from params and aliases.

TODO (priority in order):
- Button support
- Offsets
- Auto-Reconnect to PSMS
- Clean up stdout
- eta son.

**notes**
- If you remove a controller from params but not alises, osvr will still create a tracker, but it will not move, so if you want to remove a controller be sure to remove from both sections.
- Start PSMS before OSVR Server
- Please use the PSMS dll from PSMS ver. 9.0 alpha 8.2.0 or greater as I know some lower versions have funky problems.
- Properly tested with PSMS ver. 9.0 alpha 8.2.0 
- Works with OSVR-Steamvr for HMD Only, I don't think OSVR-Steamvr works with OSVR defined controllers just yet.
