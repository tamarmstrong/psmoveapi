@echo off
mkdir ..\Win32
mkdir ..\Win32\Release
copy ..\external\CLEye\x86\bin\CLEyeMulticam.dll ..\Win32\Release
copy ..\external\hidapi\windows\Release\hidapi.dll ..\Win32\Release
copy ..\external\OpenCV\x86\vc12\bin\opencv_world300.dll ..\Win32\Release
copy ..\external\GL\glut32.dll ..\Win32\Release
copy ..\external\SDL2-2.0.3\lib\x86\SDL2.dll ..\Win32\Release