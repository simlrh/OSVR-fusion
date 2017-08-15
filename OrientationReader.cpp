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
		if (orientation_paths["instantReset"].isNull()) {
			m_do_instant_reset = false;
		} else {
			m_do_instant_reset = true;
			osvrClientGetInterface(ctx, orientation_paths["instantReset"].asCString(), &m_instant_reset_path);
			m_ctx.log(OSVR_LogLevel::OSVR_LOGLEVEL_INFO, "instantReset is enabled in OSVR-Fusion.");
			m_ctx.log(OSVR_LogLevel::OSVR_LOGLEVEL_INFO, "If you notice stuttering, disable instantReset.");
		}

		m_last_yaw = 0;

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

		double a = m_alpha;
		double last_z = m_last_yaw;

		double dt = angular_v.dt;

		double z_accurate = osvrVec3GetZ(&rpy_z);
		double dzdt_fast = osvrVec3GetZ(&rpy_v) * 2 * M_PI; 		// A factor of 2*PI is missing in angularVelocity incremental quats - at least with the HDK.

		double dz_fast = dt * dzdt_fast;

		dz_fast = fixUnityAngle(dz_fast);
		z_accurate = fixUnityAngle(z_accurate);

		// Correct for the singularity at +-180 degrees
		if ((z_accurate < -M_PI / 2 && last_z > M_PI / 2) || (z_accurate > M_PI / 2 && last_z < -M_PI / 2)) {
			last_z = z_accurate;
		}

		//OSVR_TimeValue now = osvr::util::time::getNow();
		//double t_diff = osvr::util::time::duration(now, m_last_report_time);
		//m_last_report_time = now;

		OSVR_ButtonState reset_button;
		OSVR_ReturnCode resret = osvrGetButtonState(m_instant_reset_path, timeValue, &reset_button);

		OSVR_TimeValue now = osvr::util::time::getNow();
		if (reset_button == OSVR_BUTTON_PRESSED) {
			m_last_report_time = now;
		}
		double t_diff = osvr::util::time::duration(now, m_last_report_time);

		double z_displacement_limit = M_PI / 12;	// Equivalent to 15 degrees
		double z_angular_limit = M_PI / 360; // Less than half a degree per second
		double z_diff = fixUnityAngle(last_z - z_accurate);
		double z_out;

		static int boop;
		boop += 1;
		if (boop >= 250)
		{
			m_ctx.log(OSVR_LogLevel::OSVR_LOGLEVEL_INFO, "t_diff");
			m_ctx.log(OSVR_LogLevel::OSVR_LOGLEVEL_INFO, std::to_string(t_diff).c_str());
			//m_ctx.log(OSVR_LogLevel::OSVR_LOGLEVEL_INFO, "angular_limit");
			//m_ctx.log(OSVR_LogLevel::OSVR_LOGLEVEL_INFO, std::to_string(z_angular_limit).c_str());
			boop = 0;
		}

		// If the yaw difference is large enough > 30 deg but the rotation rate is small enough (< 2 deg/sec)
		// then reset filter output to the accurate yaw value.
		// This fixes the issue of "smooth" external yaw resets, making them instantaneous.
		// Cutoff values determined empirically; in the future, perhaps these values could be configurable.
		if (m_do_instant_reset && (t_diff < 1.0) && 
			((z_diff < -z_displacement_limit) || (z_diff > z_displacement_limit)) && ((dzdt_fast < z_angular_limit) && (dzdt_fast > -z_angular_limit))) {
			z_out = z_accurate;
		}
		// If the difference is smaller, implement the complementary filter.
		else {
			z_out = a*(last_z + dz_fast) + (1 - a)*(z_accurate);
		}


		// Replace bogus results with accurate yaw. Happens sometimes on startup.
		if (std::isnan(z_out)) {
			z_out = z_accurate;
		}

		// Keep the filter output nice and clean
		z_out = fixUnityAngle(z_out);

		m_last_yaw = z_out;

		OSVR_Vec3 rpy;
		osvrVec3SetX(&rpy, osvrVec3GetX(&rpy_x));
		osvrVec3SetY(&rpy, osvrVec3GetY(&rpy_y));
		osvrVec3SetZ(&rpy, z_out);

		quaternionFromRPY(&rpy, orientation);

		return OSVR_RETURN_SUCCESS;
	}

}