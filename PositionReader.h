#include "stdafx.h"

namespace je_nourish_fusion {

	class IPositionReader {
	public:
		virtual OSVR_ReturnCode update(OSVR_PositionState* position) = 0;
	};

	class PositionReaderFactory {
	public:
		static IPositionReader* getReader(OSVR_ClientContext ctx, Json::Value config);
	};

	class SinglePositionReader : public IPositionReader {
	public:
		SinglePositionReader(OSVR_ClientContext ctx, std::string position_path);
		OSVR_ReturnCode update(OSVR_PositionState* position);
	protected:
		OSVR_ClientInterface m_position;
	};

	class CombinedPositionReader : public IPositionReader {
	public:
		CombinedPositionReader(OSVR_ClientContext ctx, Json::Value position_paths);
		OSVR_ReturnCode update(OSVR_PositionState* position);
	protected:
		OSVR_ClientInterface m_positions[3];
	};

}