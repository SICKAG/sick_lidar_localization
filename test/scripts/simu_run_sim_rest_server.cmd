REM
REM Environment
REM
if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64" set PYTHON_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python36_64
if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64" set PYTHON_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64
set PATH=%PYTHON_DIR%;%PYTHON_DIR%\DLLs;%PYTHON_DIR%\Lib;%PYTHON_DIR%\Scripts;%PATH%
set PATH=c:\vcpkg\installed\x64-windows\bin;%PATH%
@if exist "c:\Anaconda3\python.exe" set PATH=c:\Anaconda3;%PATH%
@rem if exist "c:\Anaconda3\python.exe" (
@rem    set PATH=c:\Anaconda3;%PATH%
@rem   set PYTHON_DIR=c:\Anaconda3
@rem   set PYTHON_EXE=c:\Anaconda3\python.exe
@rem   set PIP_DIR=c:\Anaconda3\Scripts
@rem   set PIP_EXE=c:\Anaconda3\Scripts\pip.exe
@rem   set PATH=c:\Anaconda3\Scripts;c:\Anaconda3;%PATH%
@rem )
@echo PATH=%PATH%

REM
REM Run rest server
REM

python ../rest_server/python/sick_rest_server.py
@pause
