
REM
REM Environment
REM
rem @if exist "c:\Anaconda3\python.exe" (
rem   set PYTHON_DIR=c:\Anaconda3
rem   set PYTHON_EXE=c:\Anaconda3\python.exe
rem   set PIP_DIR=c:\Anaconda3\Scripts
rem   set PIP_EXE=c:\Anaconda3\Scripts\pip.exe
rem   set PATH=c:\Anaconda3\Scripts;c:\Anaconda3;%PATH%
rem )
@echo PATH=%PATH%

REM
REM Start rest server
REM

start "python sick_rest_server.py" python sick_rest_server.py
@timeout /t 3

REM
REM Send some http get and post requests using curl
REM

curl --version
curl -i -H "Content-Type: application/json" -X POST -d "{}" http://localhost/api/IsSystemReady
curl -i http://localhost/api/LocGetSystemState
curl -i -H "Content-Type: application/json" -X POST -d "{\"data\":{\"pose\":{\"x\":-8206,\"y\":4580,\"yaw\":85200},\"searchRadius\":1000}}" http://localhost/api/LocInitializeAtPose

@pause
