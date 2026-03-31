#pragma once

#include <string>

namespace foundation {
    
    /// @brief 错误码
enum class ErrorCode {
    kOk = 0,
    kInvalidArgument,
    kNotFound,
    kAlreadyExists,
    kImportFailed,
    kGeometryFailed,
    kPlanningFailed,
    kSerializationFailed,
    kCollisionDetected,
    kStateInvalid,
    kUnsupported,
    kInternalError
};

    /// @brief 错误信息
struct Error {
    ErrorCode code = ErrorCode::kOk;
    std::string message;
    std::string detail;
};

inline Error MakeError(ErrorCode code, std::string message, std::string detail = {})
{
    return Error{code, std::move(message), std::move(detail)};
}

} // namespace foundation