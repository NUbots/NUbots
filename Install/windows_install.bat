@ECHO off

Rem --- Extract GLee library ---
"%~dp07za465\7za.exe" x "%~dp0GLee-5.4.0-src.tar.gz" -y
"%~dp07za465\7za.exe" x "%~dp0GLee-5.4.0-src.tar" -o"%~dp0..\NUview\gl\" -y
del GLee-5.4.0-src.tar

Rem --- Extract Icons library ---
"%~dp07za465\7za.exe" x "%~dp0diagona.zip" -o"%~dp0..\NUview\diagona\" -y

Rem --- Extract zlib library ---
"%~dp07za465\7za.exe" x "%~dp0zlib-1.2.3.tar.gz" -y
"%~dp07za465\7za.exe" x "%~dp0zlib-1.2.3.tar" -y
xcopy /E /I /Y zlib-1.2.3 "%~dp0..\Tools\zlib"
del zlib-1.2.3.tar
RMDIR /s /Q zlib-1.2.3

ECHO All file extracted. Enjoy!
PAUSE