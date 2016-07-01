#include "stdafx.h"

namespace je_nourish_fusion {

	void rpyFromQuaternion(OSVR_Quaternion* quaternion, OSVR_Vec3* rpy) {
		double q[4] = {
			osvrQuatGetW(quaternion),
			osvrQuatGetZ(quaternion),
			osvrQuatGetX(quaternion),
			osvrQuatGetY(quaternion)
		};

		osvrVec3SetX(rpy, -atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])));
		osvrVec3SetY(rpy, -asin(2 * (q[0] * q[2] - q[3] * q[1])));
		osvrVec3SetZ(rpy, -atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])));
	}

	void quaternionFromRPY(OSVR_Vec3* rpy, OSVR_Quaternion* quaternion) {
		double r = -osvrVec3GetX(rpy);
		double p = -osvrVec3GetY(rpy);
		double y = -osvrVec3GetZ(rpy);

		osvrQuatSetZ(quaternion, sin(r / 2) * cos(p / 2) * cos(y / 2) - cos(r / 2) * sin(p / 2) * sin(y / 2));
		osvrQuatSetX(quaternion, cos(r / 2) * sin(p / 2) * cos(y / 2) + sin(r / 2) * cos(p / 2) * sin(y / 2));
		osvrQuatSetY(quaternion, cos(r / 2) * cos(p / 2) * sin(y / 2) - sin(r / 2) * sin(p / 2) * cos(y / 2));
		osvrQuatSetW(quaternion, cos(r / 2) * cos(p / 2) * cos(y / 2) + sin(r / 2) * sin(p / 2) * sin(y / 2));
	}

	void invertQuaternion(OSVR_Quaternion* quaternion) {
		double x = osvrQuatGetX(quaternion);
		double y = osvrQuatGetY(quaternion);
		double z = osvrQuatGetZ(quaternion);
		double w = osvrQuatGetW(quaternion);

		double magnitude = sqrt(x*x + y*y + z*z + w*w);

		osvrQuatSetX(quaternion, -osvrQuatGetX(quaternion) / magnitude);
		osvrQuatSetY(quaternion, -osvrQuatGetY(quaternion) / magnitude);
		osvrQuatSetY(quaternion, -osvrQuatGetZ(quaternion) / magnitude);

		osvrQuatSetW(quaternion, osvrQuatGetW(quaternion) / magnitude);
	}

	void multiplyQuaternion(OSVR_Quaternion* q1, OSVR_Quaternion* q2) {
		double x = osvrQuatGetX(q1) * osvrQuatGetW(q2) + osvrQuatGetY(q1) * osvrQuatGetZ(q2) - osvrQuatGetZ(q1) * osvrQuatGetY(q2) + osvrQuatGetW(q1) * osvrQuatGetX(q2);
		double y = -osvrQuatGetX(q1) * osvrQuatGetZ(q2) + osvrQuatGetY(q1) * osvrQuatGetW(q2) + osvrQuatGetZ(q1) * osvrQuatGetX(q2) + osvrQuatGetW(q1) * osvrQuatGetY(q2);
		double z = osvrQuatGetX(q1) * osvrQuatGetY(q2) - osvrQuatGetY(q1) * osvrQuatGetX(q2) + osvrQuatGetZ(q1) * osvrQuatGetW(q2) + osvrQuatGetW(q1) * osvrQuatGetZ(q2);
		double w = -osvrQuatGetX(q1) * osvrQuatGetX(q2) - osvrQuatGetY(q1) * osvrQuatGetY(q2) - osvrQuatGetZ(q1) * osvrQuatGetZ(q2) + osvrQuatGetW(q1) * osvrQuatGetW(q2);

		osvrQuatSetX(q2, x);
		osvrQuatSetY(q2, y);
		osvrQuatSetZ(q2, z);
		osvrQuatSetW(q2, w);
	}

	void offsetTranslation(OSVR_Vec3* translation_offset, OSVR_Vec3* translation) {
		osvrVec3SetX(translation, osvrVec3GetX(translation) - osvrVec3GetX(translation_offset));
		osvrVec3SetY(translation, osvrVec3GetY(translation) - osvrVec3GetY(translation_offset));
		osvrVec3SetZ(translation, osvrVec3GetZ(translation) - osvrVec3GetZ(translation_offset));
	}

	void setupAlign(OSVR_PoseState* offset, OSVR_PoseState* poseState) {
		OSVR_Vec3* translation = &(poseState->translation);
		osvrVec3SetX(&(offset->translation), osvrVec3GetX(translation));
		osvrVec3SetY(&(offset->translation), osvrVec3GetY(translation));
		osvrVec3SetZ(&(offset->translation), osvrVec3GetZ(translation));

		OSVR_Quaternion* rotation = &(poseState->rotation);
		osvrQuatSetX(&(offset->rotation), osvrQuatGetX(rotation));
		osvrQuatSetY(&(offset->rotation), osvrQuatGetY(rotation));
		osvrQuatSetZ(&(offset->rotation), osvrQuatGetZ(rotation));
		osvrQuatSetW(&(offset->rotation), osvrQuatGetW(rotation));

		invertQuaternion(&(offset->rotation));
	}

	void applyAlign(OSVR_PoseState* offset, OSVR_PoseState* poseState) {
		offsetTranslation(&(offset->translation), &(poseState->translation));
		multiplyQuaternion(&(offset->rotation), &(poseState->rotation));
	}

}