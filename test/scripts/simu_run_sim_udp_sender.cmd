REM
REM Environment
REM
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
REM Run sim udp sender
REM

python ../rest_server/python/sim_udp_sender.py
@pause
