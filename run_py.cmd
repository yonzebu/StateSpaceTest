@ECHO OFF

SETLOCAL
SET pyfile_path=%1
SET this_path=%~dp0
SET PYTHONPATH=%this_path%src\main\python\;%PYTHONPATH%
WHERE python>.\temp.txt
SET /p py_location= < .\temp.txt
DEL .\temp.txt

%py_location% %pyfile_path%

ECHO ON