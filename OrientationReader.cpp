#include "stdafx.h"
#include <iostream>

namespace je_nourish_fusion {

	IOrientationReader* OrientationReaderFactory::getReader(OSVR_ClientContext ctx, Json::Value config) {
		IOrientationReader* reader = NULL;

		if (config.isString()) {
			reader = new SingleOrientationReader(ctx, config.asString());
		}
		else if (config.isObject() && config.isMember("roll") && config.isMember("pitch") && config.isMember("yawFast")
			&& config.isMember("yawAccurate") && config.isMember("alpha")) {
			reader = new FilteredOrientationReader(ctx, config);
		}
		else if (config.isObject() && config.isMember("roll") && config.isMember("pitch") && config.isMember("yaw")) {
			reader = new CombinedOrientationReader(ctx, config);
		}

		return reader;
	}

	SingleOrientationReader::SingleOrientationReader(OSVR_ClientContext ctx, std::string orientation_path) {
		osvrClientGetInterface(ctx, orientation_path.c_str(), &m_orientation);
	}
	OSVR_ReturnCode SingleOrientationReader::update(OSVR_OrientationState* orientation, OSVR_TimeValue* timeValue) {
		return osvrGetOrientationState(m_orientation, timeValue, orientation);
	}

	CombinedOrientationReader::CombinedOrientationReader(OSVR_ClientContext ctx, Json::Value orientation_paths) {
		osvrClientGetInterface(ctx, orientation_paths["roll"].asCString(), &(m_orientations[0]));
		osvrClientGetInterface(ctx, orientation_paths["pitch"].asCString(), &(m_orientations[1]));
		osvrClientGetInterface(ctx, orientation_paths["yaw"].asCString(), &(m_orientations[2]));
	}

	FilteredOrientationReader::FilteredOrientationReader(OSVR_ClientContext ctx, Json::Value orientation_paths) : m_ctx(ctx) {
		osvrClientGetInterface(ctx, orientation_paths["roll"].asCString(), &(m_orientations[0]));
		osvrClientGetInterface(ctx, orientation_paths["pitch"].asCString(), &(m_orientations[1]));
		osvrClientGetInterface(ctx, orientation_paths["yawFast"].asCString(), &(m_orientations[2]));
		osvrClientGetInterface(ctx, orientation_paths["yawAccurate"].asCString(), &(m_orientations[3]));
		m_alpha = orientation_paths["alpha"].asDouble();
		if (orientation_paths["recenterButton"].isNull()) {
			m_do_instant_reset = false;
		}
		else {
			m_do_instant_reset = true;
			osvrClientGetInterface(ctx, orientation_paths["recenterButton"].asCString(), &m_instant_reset_path);
			m_ctx.log(OSVR_LogLevel::OSVR_LOGLEVEL_INFO, "Recenter button detection is enabled in OSVR-Fusion.");
		}

		m_last_yaw = 0;
		m_reset_press_time = osvr::util::time::getNow();

		m_ctx.log(OSVR_LogLevel::OSVR_LOGLEVEL_INFO, "Initialized a complementary fusion filter.");
	}

	OSVR_ReturnCode CombinedOrientationReader::update(OSVR_OrientationState* orientation, OSVR_TimeValue* timeValue) {
		OSVR_OrientationState orientation_x;
		OSVR_OrientationState orientation_y;
		OSVR_OrientationState orientation_z;

		OSVR_ReturnCode xret = osvrGetOrientationState(m_orientations[0], timeValue, &orientation_x);
		OSVR_ReturnCode yret = osvrGetOrientationState(m_orientations[1], timeValue, &orientation_y);
		OSVR_ReturnCode zret = osvrGetOrientationState(m_orientations[2], timeValue, &orientation_z);

		OSVR_Vec3 rpy_x;
		OSVR_Vec3 rpy_y;
		OSVR_Vec3 rpy_z;

		rpyFromQuaternion(&orientation_x, &rpy_x);
		rpyFromQuaternion(&orientation_y, &rpy_y);
		rpyFromQuaternion(&orientation_z, &rpy_z);

		OSVR_Vec3 rpy;
		osvrVec3SetX(&rpy, osvrVec3GetX(&rpy_x));
		osvrVec3SetY(&rpy, osvrVec3GetY(&rpy_y));
		osvrVec3SetZ(&rpy, osvrVec3GetZ(&rpy_z));

		quaternionFromRPY(&rpy, orientation);

		return OSVR_RETURN_SUCCESS;
	}

	OSVR_ReturnCode FilteredOrientationReader::update(OSVR_OrientationState* orientation, OSVR_TimeValue* timeValue) {
		OSVR_OrientationState orientation_x;
		OSVR_OrientationState orientation_y;
		OSVR_OrientationState orientation_z;
		OSVR_AngularVelocityState angular_v;

		OSVR_ReturnCode xret = osvrGetOrientationState(m_orientations[0], timeValue, &orientation_x);
		OSVR_ReturnCode yret = osvrGetOrientationState(m_orientations[1], timeValue, &orientation_y);
		OSVR_ReturnCode zret = osvrGetOrientationState(m_orientations[3], timeValue, &orientation_z);
		OSVR_ReturnCode angret = osvrGetAngularVelocityState(m_orientations[2], timeValue, &angular_v);

		OSVR_Vec3 rpy_x;
		OSVR_Vec3 rpy_y;
		OSVR_Vec3 rpy_z;
		OSVR_Vec3 rpy_v;

		rpyFromQuaternion(&orientation_x, &rpy_x);
		rpyFromQuaternion(&orientation_y, &rpy_y);
		rpyFromQuaternion(&orientation_z, &rpy_z);
		rpyFromQuaternion(&angular_v.incrementalRotation, &rpy_v);

		double a = m_alpha;				// Grab filter threshold variable
		double last_z = m_last_yaw;		// Grab last yaw
		double dt = angular_v.dt;		// Grab timestep for use with angular velocity. Strangely, not the same as time between timeValue args

		double z_accurate = osvrVec3GetZ(&rpy_z);				// Grab accurate yaw value
		double dzdt_fast = osvrVec3GetZ(&rpy_v) * 2 * M_PI; 	// Grab fast yaw rate. A factor of 2*PI is missing in angularVelocity incremental quats - at least with the HDK.
		double dz_fast = dt * dzdt_fast;						// Create fast yaw incremental value by multiplying by timestep
		double z_fast = last_z + dz_fast;						// Create fast yaw value.

		// Clean up input angles
		z_fast = fixAngleWrap(z_fast);
		z_accurate = fixAngleWrap(z_accurate);

		// Handle angle wrap discrepancies (prevents the filter from spinning wrong way)
		if (z_accurate < -M_PI / 2 && z_fast > M_PI / 2) { z_fast -= 2 * M_PI; }
		if (z_accurate > M_PI / 2 && z_fast < -M_PI / 2) { z_fast += 2 * M_PI; }

		double z_displacement_limit = M_PI / 18;		// Equivalent to 10 degrees
		double z_angular_limit = M_PI / 180;			// Rotation less than one degree per second
		double z_diff = fixAngleWrap(last_z - z_accurate);
		double z_out;

		// Read the instantReset button
		OSVR_ButtonState reset_button;
		OSVR_ReturnCode resret = osvrGetButtonState(m_instant_reset_path, timeValue, &reset_button);

		// If instantReset is enabled, store the reset button press timestamp for later comparison
		double button_time_diff;
		if (m_do_instant_reset) {
			OSVR_TimeValue now = osvr::util::time::getNow();
			if (reset_button == OSVR_BUTTON_PRESSED) {
				m_reset_press_time = now;
			}
			button_time_diff = osvr::util::time::duration(now, m_reset_press_time);
		}

		// If instantReset is enabled, then perform some checks
		// Is the difference between the previous yaw value and the current one sufficiently large?
		// Is the angular velocity low?
		// Was the reset button recently or currently pressed?
		// If all conditions are true, then we probably detected a yaw reset, so make it snappy.
		if (m_do_instant_reset && (button_time_diff < 0.5) &&
			((z_diff < -z_displacement_limit) || (z_diff > z_displacement_limit)) && ((dzdt_fast < z_angular_limit) && (dzdt_fast > -z_angular_limit))) {
			z_out = z_accurate;
		}
		// If instantReset is not enabled or if the checks are not met, carry on with regular filter implementation.
		else {
			z_out = a*(z_fast)+(1 - a)*(z_accurate);
		}

		// Replace bogus results with accurate yaw. Happens sometimes on startup.
		if (std::isnan(z_out)) {
			z_out = z_accurate;
		}

		// Clean up the output angle just in case
		z_out = fixAngleWrap(z_out);

		// Store new value for next filter iteration
		m_last_yaw = z_out;

		// Report the new orientation
		OSVR_Vec3 rpy;
		osvrVec3SetX(&rpy, osvrVec3GetX(&rpy_x));
		osvrVec3SetY(&rpy, osvrVec3GetY(&rpy_y));
		osvrVec3SetZ(&rpy, z_out);

		quaternionFromRPY(&rpy, orientation);

		return OSVR_RETURN_SUCCESS;
	}

}