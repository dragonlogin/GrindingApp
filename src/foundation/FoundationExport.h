#pragma once

// 核心：跨平台 动态库 导入导出宏
// Windows 平台：需要 __declspec(dllexport/dllimport)
// Linux/macOS 平台：无需符号修饰，宏定义为空
#if defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
    #ifdef FOUNDATION_EXPORT_DLL
        #define FOUNDATION_EXPORT __declspec(dllexport)
    #else
        #define FOUNDATION_EXPORT __declspec(dllimport)
    #endif
#else
    #define FOUNDATION_EXPORT
#endif