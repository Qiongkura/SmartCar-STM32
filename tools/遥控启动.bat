@echo off
chcp 65001 >nul
title 智能小车 蓝牙遥控 (模式七)
cd /d "%~dp0"

set "PY="
python -c "" >nul 2>nul && set "PY=python"
if not defined PY py -3 -c "" >nul 2>nul && set "PY=py -3"
if not defined PY (
    echo [!] 没找到 Python, 请先安装 Python 3 并勾选 Add to PATH
    pause
    exit /b 1
)

%PY% rc_send.py

pause

