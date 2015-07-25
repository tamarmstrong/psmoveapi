@echo off
mkdir ..\x64
mkdir ..\x64\Release
copy ..\..\SDL2-2.0.3\lib\x64\SDL2.dll ..\x64\Release
copy ..\..\libusb-1.0.19\x64\Release\dll\libusb-1.0.dll ..\x64\Release
