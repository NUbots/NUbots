@ECHO off

ECHO --- Extracting Icons library ---
"%~dp07za465\7za.exe" x "%~dp0diagona.zip" -o"%~dp0..\NUview\diagona\" -y

ECHO All file extracted. Enjoy!
PAUSE