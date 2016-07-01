#pragma once

// Internal Includes
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>
#include <osvr/AnalysisPluginKit/AnalysisPluginKitC.h>
#include <osvr/ClientKit/Context.h>
#include <osvr/ClientKit/Interface.h>
#include <osvr/ClientKit/InterfaceStateC.h>
#include <osvr/PluginKit/DeviceInterface.h>
#include <osvr/Util/EigenInterop.h>

// Generated JSON header file
#include "je_nourish_fusion_json.h"

// Library/third-party includes
#include <json/json.h>

// Standard includes
#include <string>
#include <cmath>

#include "FusionMath.h"
#include "PositionReader.h"
#include "OrientationReader.h"