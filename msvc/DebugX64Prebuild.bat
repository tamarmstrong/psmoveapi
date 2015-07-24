@echo off
mkdir ..\x64
mkdir ..\x64\Debug
copy ..\external\hidapi\windows\x64\Debug\hidapi.dll ..\x64\Debug
copy ..\external\hidapi\windows\x64\Debug\hidapi.pdb ..\x64\Debug
copy ..\external\OpenCV\x64\vc12\bin\opencv_world300d.dll ..\x64\Debug
copy ..\external\GL\glut.dll ..\x64\Debug
copy ..\external\SDL2-2.0.3\lib\x64\SDL2.dll ..\x64\Debug