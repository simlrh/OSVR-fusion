#include "stdafx.h"

namespace je_nourish_fusion {

	void rpyFromQuaternion(OSVR_Quaternion* quaternion, OSVR_Vec3* rpy);
	void quaternionFromRPY(OSVR_Vec3* rpy, OSVR_Quaternion* quaternion);
	double fixUnityAngle(double angle);

}