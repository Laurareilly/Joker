::Utility script to help you prepare a clean project 
::directory for ALL SUBMISSIONS.
::By Dan Buckstein
@echo off
echo Preparing submission...
set /p outputdir=" Please type output folder name: "
set rootpath=..\..
set projectroot=%1
set projectsuff=%2
set outputpath=_SUBMISSION_%projectroot%\%outputdir%_%projectroot%
mkdir %outputpath%
cd %outputpath%

::build directory tree
::DO NOT ADD TO THIS LIST
set includedst=include
set resourcedst=resource
set sourcedst=source
set projectdst=project\%projectroot%
mkdir %includedst%
mkdir %resourcedst%
mkdir %sourcedst%\egpfw
mkdir %sourcedst%\egpfwDemo
mkdir %projectdst%\egpfw
mkdir %projectdst%\egpfwDemo%projectsuff%

::COPY RELEVANT FILES ONLY
::DO NOT ADD TO THIS LIST
set includedir=%rootpath%\%includedst%
set resourcedir=%rootpath%\%resourcedst%
set sourcedir=%rootpath%\%sourcedst%
set projectdir=%rootpath%\%projectdst%
rmdir /s /q %projectdir%\egpfw\build
rmdir /s /q %projectdir%\egpfwDemo%projectsuff%\build
xcopy /y /s %projectdir%\egpfw\*					.\%projectdst%\egpfw\
xcopy /y /s %projectdir%\egpfwDemo%projectsuff%\*	.\%projectdst%\egpfwDemo%projectsuff%\
del /s /q .\%projectdst%\egpfwDemo%projectsuff%\egpfwDemo%projectsuff%\*.aps
xcopy /y /s %sourcedir%\egpfw\*						.\%sourcedst%\egpfw\
xcopy /y /s %sourcedir%\egpfwDemo\*					.\%sourcedst%\egpfwDemo\
xcopy /y /s %includedir% 							.\%includedst%\
xcopy /y /s %resourcedir% 							.\%resourcedst%\

::done
pause