#include "stdafx.h"

namespace je_nourish_fusion {

	class IOrientationReader {
	public:
		virtual	OSVR_ReturnCode update(OSVR_OrientationState* orientation, OSVR_TimeValue* timeValue) = 0;
	};

	class OrientationReaderFactory {
	public:
		static IOrientationReader* getReader(OSVR_ClientContext ctx, Json::Value config);
	};

	class SingleOrientationReader : public IOrientationReader {
	public:
		SingleOrientationReader(OSVR_ClientContext ctx, std::string orientation_path);
		OSVR_ReturnCode update(OSVR_OrientationState* orientation, OSVR_TimeValue* timeValue);
	protected:
		OSVR_ClientInterface m_orientation;
	};

	class CombinedOrientationReader : public IOrientationReader {
	public:
		CombinedOrientationReader(OSVR_ClientContext ctx, Json::Value orientation_paths);
		OSVR_ReturnCode update(OSVR_OrientationState* orientation, OSVR_TimeValue* timeValue);
	protected:
		OSVR_ClientInterface m_orientations[3];
	};

}