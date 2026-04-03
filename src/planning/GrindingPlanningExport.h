#pragma once

#if defined(_WIN32)
#ifdef GRINDING_PLANNING_BUILDING_DLL
#define GRINDING_PLANNING_EXPORT __declspec(dllexport)
#else
#define GRINDING_PLANNING_EXPORT __declspec(dllimport)
#endif
#else
#define GRINDING_PLANNING_EXPORT
#endif