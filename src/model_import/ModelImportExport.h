#pragma once

#if defined(_WIN32) || defined(_WIN64)
  #ifdef MODEL_IMPORT_BUILDING_DLL
    #define MODEL_IMPORT_EXPORT __declspec(dllexport)
  #else
    #define MODEL_IMPORT_EXPORT __declspec(dllimport)
  #endif
#else
  #define MODEL_IMPORT_EXPORT
#endif