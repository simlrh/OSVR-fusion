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

	double fixUnityAngle(double angle)
	{
		if (angle > M_PI) {
			angle = (2 * M_PI) - angle;
		}
		else if (angle < -M_PI) {
			angle = (2 * M_PI) + angle;
		}
		return angle;
	}

}