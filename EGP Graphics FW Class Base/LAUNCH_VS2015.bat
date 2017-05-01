@echo off
set EGPFW_DIR=%~dp0
call "%EGPFW_DIR%\fw_helpers\win\egpfw_win_launch.bat" "%VS140COMNTOOLS%..\IDE\" "%EGPFW_DIR%" VS2015 _SLN EGP-Graphics-FW