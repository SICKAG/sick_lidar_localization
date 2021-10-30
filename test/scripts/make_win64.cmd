REM 
REM Build sick_lidar_localization on native Windows (no ROS required)
REM

pushd ..\..

call "%ProgramFiles(x86)%\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64
set PATH=%ProgramFiles%\CMake\bin;%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64;%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64;%PATH%
set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%

set _os=x64
set _cmake_string=Visual Studio 16
set _msvc=Visual Studio 2019
set _cmake_build_dir=build

REM 
REM Build sick_lidar_localization on native Windows (no ROS required)
REM 

if not exist %_cmake_build_dir% mkdir %_cmake_build_dir%
pushd %_cmake_build_dir%
cmake -DROS_VERSION=0 -G "%_cmake_string%" ..
if %ERRORLEVEL% neq 0 ( @echo ERROR building %_cmake_string% sick_lidar_localization with cmake & @pause )
start "sick_lidar_localization.sln" sick_lidar_localization.sln
popd
popd
@timeout /t 10
