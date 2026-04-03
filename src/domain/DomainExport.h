#pragma once

#if defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
    #ifdef DOMAIN_EXPORT_DLL
        #define DOMAIN_EXPORT __declspec(dllexport)
    #else
        #define DOMAIN_EXPORT __declspec(dllimport)
    #endif
#else
    #define DOMAIN_EXPORT
#endif
