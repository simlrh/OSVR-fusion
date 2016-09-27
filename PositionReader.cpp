#include "stdafx.h"

namespace je_nourish_fusion {

	IPositionReader* PositionReaderFactory::getReader(OSVR_ClientContext ctx, Json::Value config) {
		IPositionReader* reader = NULL;

		if (config.isString()) {
			reader = new SinglePositionReader(ctx, config.asString());
		}
		if (config.isObject() && config.isMember("x") && config.isMember("y") && config.isMember("z")) {
			reader = new CombinedPositionReader(ctx, config);
		}

		return reader;
	}

	SinglePositionReader::SinglePositionReader(OSVR_ClientContext ctx, std::string position_path) {
		osvrClientGetInterface(ctx, position_path.c_str(), &m_position);
	}
	OSVR_ReturnCode SinglePositionReader::update(OSVR_PositionState* position, OSVR_TimeValue* timeValue) {
		return osvrGetPositionState(m_position, timeValue, position);
	}

	CombinedPositionReader::CombinedPositionReader(OSVR_ClientContext ctx, Json::Value position_paths) {
		osvrClientGetInterface(ctx, position_paths["x"].asCString(), &(m_positions[0]));
		osvrClientGetInterface(ctx, position_paths["y"].asCString(), &(m_positions[1]));
		osvrClientGetInterface(ctx, position_paths["z"].asCString(), &(m_positions[2]));
	}
	OSVR_ReturnCode CombinedPositionReader::update(OSVR_PositionState* position, OSVR_TimeValue* timeValue) {
		OSVR_PositionState position_x;
		OSVR_PositionState position_y;
		OSVR_PositionState position_z;

		OSVR_ReturnCode xret = osvrGetPositionState(m_positions[0], timeValue, &position_x);
		OSVR_ReturnCode yret = osvrGetPositionState(m_positions[1], timeValue, &position_y);
		OSVR_ReturnCode zret = osvrGetPositionState(m_positions[2], timeValue, &position_z);

		if (xret == OSVR_RETURN_SUCCESS) {
			osvrVec3SetX(position, osvrVec3GetX(&position_x));
		}
		if (yret == OSVR_RETURN_SUCCESS) {
			osvrVec3SetY(position, osvrVec3GetY(&position_y));
		}
		if (zret == OSVR_RETURN_SUCCESS) {
			osvrVec3SetZ(position, osvrVec3GetZ(&position_z));
		}

		return OSVR_RETURN_SUCCESS;
	}

}