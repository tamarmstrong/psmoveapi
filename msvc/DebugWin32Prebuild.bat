@echo off
mkdir ..\Win32
mkdir ..\Win32\Debug
copy ..\external\CLEye\x86\bin\CLEyeMulticam.dll ..\Win32\Debug
copy ..\external\hidapi\windows\Debug\hidapi.dll ..\Win32\Debug
copy ..\external\hidapi\windows\Debug\hidapi.pdb ..\Win32\Debug
copy ..\external\OpenCV\x86\vc12\bin\opencv_world300d.dll ..\Win32\Debug
copy ..\external\GL\glut32.dll ..\Win32\Debug
copy ..\external\SDL2-2.0.3\lib\x86\SDL2.dll ..\Win32\Debug