@ECHO off

ECHO --- Extracting GLee library ---
"%~dp07za465\7za.exe" x "%~dp0GLee-5.4.0-src.tar.gz" -y
"%~dp07za465\7za.exe" x "%~dp0GLee-5.4.0-src.tar" -o"%~dp0..\NUview\gl\" -y
del GLee-5.4.0-src.tar

ECHO --- Extracting Icons library ---
"%~dp07za465\7za.exe" x "%~dp0diagona.zip" -o"%~dp0..\NUview\diagona\" -y

ECHO All file extracted. Enjoy!
PAUSE