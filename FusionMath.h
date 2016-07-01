#include "stdafx.h"

namespace je_nourish_fusion {

	void rpyFromQuaternion(OSVR_Quaternion* quaternion, OSVR_Vec3* rpy);
	void quaternionFromRPY(OSVR_Vec3* rpy, OSVR_Quaternion* quaternion);

	void invertQuaternion(OSVR_Quaternion* quaternion);
	void multiplyQuaternion(OSVR_Quaternion* q1, OSVR_Quaternion* q2);

	void offsetTranslation(OSVR_Vec3* translation_offset, OSVR_Vec3* translation);

	void setupAlign(OSVR_PoseState* offset, OSVR_PoseState* poseState);
	void applyAlign(OSVR_PoseState* offset, OSVR_PoseState* poseState);

}