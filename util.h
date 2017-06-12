#pragma once
#include <PSMoveClient_CAPI.h>
#include <string>

#define PRNT_PFX " [OSVR Move]"

namespace util {
	std::string get_psm_error_str(PSMResult result) {
		switch (result) {
		case PSMResult_Canceled:
			return "Connection cancelled";
		case PSMResult_Error:
			return "Connection error";
		case PSMResult_NoData:
			return "No data received";
		case PSMResult_Timeout:
			return "Connection timed out";
		default:
			return "Unknown error code";
		}
	}
}