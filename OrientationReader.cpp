#include "stdafx.h"
#include <iostream>

namespace je_nourish_fusion {

	IOrientationReader* OrientationReaderFactory::getReader(OSVR_ClientContext ctx, Json::Value config) {
		IOrientationReader* reader = NULL;

		if (config.isString()) {
			reader = new SingleOrientationReader(ctx, config.asString());
		}
		if (config.isObject() && config.isMember("roll") && config.isMember("pitch") && config.isMember("yaw")) {
			reader = new CombinedOrientationReader(ctx, config);
		}

		return reader;
	}

	SingleOrientationReader::SingleOrientationReader(OSVR_ClientContext ctx, std::string orientation_path) {
		osvrClientGetInterface(ctx, orientation_path.c_str(), &m_orientation);
	}
	OSVR_ReturnCode SingleOrientationReader::update(OSVR_OrientationState* orientation) {
		OSVR_TimeValue timestamp;

		return osvrGetOrientationState(m_orientation, &timestamp, orientation);
	}

	CombinedOrientationReader::CombinedOrientationReader(OSVR_ClientContext ctx, Json::Value orientation_paths) {
		osvrClientGetInterface(ctx, orientation_paths["roll"].asCString(), &(m_orientations[0]));
		osvrClientGetInterface(ctx, orientation_paths["pitch"].asCString(), &(m_orientations[1]));
		osvrClientGetInterface(ctx, orientation_paths["yaw"].asCString(), &(m_orientations[2]));
	}

	OSVR_ReturnCode CombinedOrientationReader::update(OSVR_OrientationState* orientation) {
		OSVR_OrientationState orientation_x;
		OSVR_OrientationState orientation_y;
		OSVR_OrientationState orientation_z;

		OSVR_TimeValue timestamp;

		OSVR_ReturnCode xret = osvrGetOrientationState(m_orientations[0], &timestamp, &orientation_x);
		OSVR_ReturnCode yret = osvrGetOrientationState(m_orientations[1], &timestamp, &orientation_y);
		OSVR_ReturnCode zret = osvrGetOrientationState(m_orientations[2], &timestamp, &orientation_z);

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

}