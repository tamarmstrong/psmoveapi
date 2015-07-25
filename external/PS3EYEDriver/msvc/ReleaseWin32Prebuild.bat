@echo off
mkdir ..\Win32
mkdir ..\Win32\Release
copy ..\..\SDL2-2.0.3\lib\x86\SDL2.dll ..\Win32\Release
copy ..\..\libusb-1.0.19\Win32\Release\dll\libusb-1.0.dll ..\Win32\Release