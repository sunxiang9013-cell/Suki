@echo off
chcp 65001 >nul
cd /d "C:\Users\Administrator\Desktop\轮椅项目\Coding\tools"
set "NODE_EXE=C:\Program Files\nodejs\node.exe"
set "URL=http://127.0.0.1:8123/serial_debug.html"
if not exist "%NODE_EXE%" (
  echo Cannot find Node.js: %NODE_EXE%
  pause
  exit /b 1
)
start "eye-calibration-server" /min "%NODE_EXE%" local_server.js
timeout /t 2 /nobreak >nul
start "" "%URL%"
exit /b
