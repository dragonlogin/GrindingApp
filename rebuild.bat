@echo off
call "D:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat"
"D:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -S E:\Code\GrindingApp -B E:\Code\GrindingApp\build -DVCPKG_MANIFEST_INSTALL=OFF
echo CMAKE_EXIT=%errorlevel%
