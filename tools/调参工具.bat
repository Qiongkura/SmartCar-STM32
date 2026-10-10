@echo off
chcp 65001 >nul
title 小车调参工具
cd /d "%~dp0"
python "%~dp0调参工具.py"
if errorlevel 1 pause

