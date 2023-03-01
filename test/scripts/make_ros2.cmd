REM 
REM Build sick_lidar_localization on Windows ROS-2
REM

pushd ..\..\..\..
rem Workaround for "The fully qualified file name must be less than 260 characters" errors:
rem Copy folder sick_lidar_localization2_pretest to c:\<short_folder_name> and run in that directory...
rem if not exist r:\sick_lidar_localization2_pretest subst r: ..
rem pushd r:\sick_lidar_localization2_pretest
rem Cleanup
rmdir /s/q .\log
for %%i in ( .\install\sick_lidar_localization\lib .\install\sick_lidar_localization\lib\sick_lidar_localization .\build\sick_lidar_localization\Debug .\build\sick_lidar_localization\Release ) do (
  if exist %%i\gen_service_call.exe        del /f/q %%i\gen_service_call.exe
  if exist %%i\sick_lidar_localization.exe del /f/q %%i\sick_lidar_localization.exe
)

call "%ProgramFiles(x86)%\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64
if exist c:\dev\ros2_foxy\local_setup.bat call c:\dev\ros2_foxy\local_setup.bat
if exist c:\opt\ros\foxy\x64\setup.bat call c:\opt\ros\foxy\x64\setup.bat
set PATH=%ProgramFiles%\CMake\bin;%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64;%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64;%PATH%
set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%

REM 
REM Build sick_lidar_localization on Windows with colcon for ROS2
REM 

if exist .\src\sick_lidar_localization2_pretest\package_ros2.xml copy /b/y .\src\sick_lidar_localization2_pretest\package_ros2.xml .\src\sick_lidar_localization2_pretest\package.xml
if exist .\src\sick_lidar_localization\package_ros2.xml copy /b/y .\src\sick_lidar_localization\package_ros2.xml .\src\sick_lidar_localization\package.xml
colcon build --cmake-args " -DROS_VERSION=2" --event-handlers console_direct+
call .\install\setup.bat
start "sick_lidar_localization.sln" .\build\sick_lidar_localization\sick_lidar_localization.sln

@timeout /t 3
@echo.
if not exist .\build\sick_lidar_localization\Release\gen_service_call.exe        ( @echo colcon build gen_service_call.exe failed        & @pause ) else ( @echo Successfully build gen_service_call.exe for ROS-2 Windows )
if not exist .\build\sick_lidar_localization\Release\sick_lidar_localization.exe ( @echo colcon build sick_lidar_localization.exe failed & @pause ) else ( @echo Successfully build sick_lidar_localization.exe for ROS-2 Windows )

popd
popd
@pause
rem @timeout /t 10
