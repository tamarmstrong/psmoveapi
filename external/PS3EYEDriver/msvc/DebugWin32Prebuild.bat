@echo off
mkdir ..\Win32
mkdir ..\Win32\Debug
copy ..\..\SDL2-2.0.3\lib\x86\SDL2.dll ..\Win32\Debug
copy ..\..\libusb-1.0.19\Win32\Debug\dll\libusb-1.0.dll ..\Win32\Debug