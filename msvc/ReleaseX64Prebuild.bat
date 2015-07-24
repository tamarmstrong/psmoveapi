@echo off
mkdir ..\x64
mkdir ..\x64\Release
copy ..\external\hidapi\windows\x64\Release\hidapi.dll ..\x64\Release
copy ..\external\hidapi\windows\x64\Release\hidapi.pdb ..\x64\Release
copy ..\external\OpenCV\x64\vc12\bin\opencv_world300.dll ..\x64\Release
copy ..\external\GL\glut.dll ..\x64\Release
copy ..\external\SDL2-2.0.3\lib\x64\SDL2.dll ..\x64\Release