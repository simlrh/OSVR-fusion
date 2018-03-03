# OSVR-fusion [![Donate](https://nourish.je/assets/images/donate.svg)](http://ko-fi.com/A250KJT)

An OSVR plugin that creates trackers from different sources of data. For example, taking the orientation data from an HMD with no positional tracking, and the position data from a motion controller with no orientation data.

It can also combine axes from different trackers, eg taking pitch and roll from an accelerometer and yaw from a magnetometer, or x and y position from a video tracker and z position from a depth camera.

A complementary filter can be used to smoothly merge yaw data from one device with another. For example, a faster gyroscopic yaw reading from one device can be combined with a more accurate video tracker yaw reading from another device.

Build following the [standard OSVR plugin build instructions](http://resource.osvr.com/docs/OSVR-Core/TopicWritingDevicePlugin.html).

## Tracker alignment

The orientation and position data will likely be misaligned, eg, you are facing forward and leaning forward, but your tracked position instead moves to the side. To correct this, align the orientation tracker with the position tracker's axes and run osvr_reset_yaw on the orientation tracker.

For example, with the OSVR HDK and a Kinect, you would place the HDK in front of the Kinect, pointing towards it, then run

    osvr_reset_yaw.exe --path "/com_osvr_Multiserver/OSVRHackerDevKitPrediction0/semantic/hmd"

This replaces the `alignInitialOrientation` option in previous versions.

## Maintainers

[nanospork](https://github.com/nanospork) and [simlrh](https://github.com/simlrh)
	
## Sample osvr_server_config.json:

    {
    	"drivers": [
			// Combine Oculus DK1 orientation with Kinect position
			{
    			"plugin": "je_nourish_fusion",
    			"driver": "FusionDevice",
    			"params": {
    				"name": "DK1_Kinectv2",
    				"position": "/je_nourish_kinect/KinectV2/semantic/body1/head",
    				"orientation": "/je_nourish_openhmd/Oculus Rift (Devkit)/semantic/hmd",
					// Eyes are above and in front of the center of the head
    				"offsetFromRotationCenter": {
    					"x": 0,
    					"y": 0.01,
    					"z": -0.05
    				},
					// Pass the timestamp from the Kinect skeleton data to OSVR
					"timestamp": "position"
    			}
    		},
			// Combine more accurate pitch and roll from Wii Nunchuk with yaw and position from Kinect
    		{
    			"plugin": "je_nourish_fusion",
    			"driver": "FusionDevice",
    			"params": {
    				"name": "Wii_Kinect_Right",
    				"position": "/je_nourish_kinect/KinectV2/semantic/body1/arms/right/hand",
    				"orientation": {
    					"roll": "/je_nourish_wiimote/WiimoteDevice/semantic/wiimote1/nunchuk",
    					"pitch": "/je_nourish_wiimote/WiimoteDevice/semantic/wiimote1/nunchuk",
    					"yaw": "/je_nourish_kinectv2/KinectV2/semantic/body1/arms/right/hand"
    				}
    			}
    		},
			// Use the complementary filter to improve the yaw performance of a home-made gyro device
    		{
    			"plugin": "je_nourish_fusion",
    			"driver": "FusionDevice",
    			"params": {
    				"name": "Gyro_Kinect_Left",
    				"position": "/je_nourish_kinect/KinectV2/semantic/body1/arms/left/hand",
    				"orientation": {
    					"roll": "/my-gyro-device/semantic/controller/left",
    					"pitch": "/my-gyro-device/semantic/controller/left",
    					// Gyro yaw updates faster, but Kinect yaw is more accurate over time (doesn't drift)
						"yawFast": "/my-gyro-device/semantic/controller/left",
						"yawAccurate": "/je_nourish_kinectv2/KinectV2/semantic/body1/arms/left/hand",
						// Alpha must be between 0 and 1. Usually > 0.95, favoring speed.
						"alpha": 0.99
    				},
					// Pass the faster timestamp from the gyro device to OSVR.
					"timestamp": "rotation"
    			}
    		}
		],
    	"aliases": {
    		"/me/head": "/je_nourish_fusion/DK1_Kinectv2/tracker/0",
    		"/me/hands/right": "/je_nourish_fusion/Wii_Kinect_Right/tracker/0",
			"/me/hands/left": "/je_nourish_fusion/Gyro_Kinect_Left/tracker/0"
    	}
    }