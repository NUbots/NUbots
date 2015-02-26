@ECHO OFF

set PWD=%~dp0

cd /d "%PWD%"

boot2docker init
boot2docker up --vbox-share="%PWD%=nubots"
boot2docker ssh "sudo mkdir -p /nubots && sudo mount -t vboxsf nubots /nubots"
echo You can now SSH to:
boot2docker ip
pause
