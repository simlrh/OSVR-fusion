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

	class FilteredOrientationReader : public IOrientationReader {
	public:
		FilteredOrientationReader(OSVR_ClientContext ctx, Json::Value orientation_paths);
		OSVR_ReturnCode update(OSVR_OrientationState* orientation, OSVR_TimeValue* timeValue);
	protected:
		OSVR_ClientInterface m_orientations[4];
		osvr::clientkit::ClientContext m_ctx;
		bool m_do_instant_reset;
		OSVR_ClientInterface m_instant_reset_path;
		OSVR_TimeValue m_reset_press_time;
		double m_alpha;
		double m_last_yaw;
	};

}