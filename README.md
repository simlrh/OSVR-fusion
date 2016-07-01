# OSVR-fusion

An OSVR plugin that creates trackers from different sources of data. For example, taking the orientation data from an HMD with no positional tracking, and the position data from a motion controller with no orientation data.

It can also combine axes from different trackers, eg taking pitch and roll from an accelerometer and yaw from a magnetometer, or x and y position from a video tracker and z position from a depth camera.

    git clone https://github.com/simlrh/OSVR-fusion
    cd OSVR-fusion
    git submodule init
    git submodule update

Then follow the standard OSVR plugin build instructions.

Sample osvr_server_config.json:

    {
    	"drivers": [
			// Combine Oculus DK1 orientation with Kinect position
			{
    			"plugin": "je_nourish_fusion",
    			"driver": "FusionDevice",
    			"params": {
    				"name": "DK1_Kinectv2",
    				"position": "/je_nourish_kinectv2/KinectV2/semantic/body1/head",
    				"orientation": "/je_nourish_openhmd/Oculus Rift (Devkit)/semantic/hmd",
					// Align DK1 and Kinect axes (point hmd directly at kinect on startup)
    				"alignInitialOrientation": true,
					// Eyes are above and in front of the center of the head
    				"offsetFromRotationCenter": {
    					"x": 0,
    					"y": 0.01,
    					"z": -0.05
    				}
    			}
    		},
			// Combine more accurate pitch and roll from Wii Nunchuk with yaw and position from Kinect
    		{
    			"plugin": "je_nourish_fusion",
    			"driver": "FusionDevice",
    			"params": {
    				"name": "Wii_Kinect_Right",
    				"position": "/je_nourish_kinectv2/KinectV2/semantic/body1/arms/right/hand",
    				"orientation": {
    					"roll": "/je_nourish_wiimote/WiimoteDevice/semantic/wiimote1/nunchuk",
    					"pitch": "/je_nourish_wiimote/WiimoteDevice/semantic/wiimote1/nunchuk",
    					"yaw": "/je_nourish_kinectv2/KinectV2/semantic/body1/arms/right/hand"
    				}
    			}
    		}
		],
    	"aliases": {
    		"/me/head": "/je_nourish_fusion/DK1_Kinectv2/tracker/0",
    		"/me/hands/right": "/je_nourish_fusion/Wii_Kinect_Right/tracker/0"
    	}
    }