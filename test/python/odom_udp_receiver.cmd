@if exist %ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64 (
  set PYTHON_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Shared\Python37_64
  set PATH=%PYTHON_DIR%;%PYTHON_DIR%\Scripts;%PATH%
)

python odom_udp_receiver.py
