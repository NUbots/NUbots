# Systemd Unit files for purging docker objects

Sets up a systemd timer and service for monitoring hard drive use and removing docker objects (images, containers, volumes, networks) when a usage threshold has been exceeded.

## Install

1. Download systemd files into systemd folder
```
mkdir -p "${HOME}/.config/systemd/user"
wget https://raw.githubusercontent.com/NUbots/NUbots/master/doc/BuildKiteDockerClean/docker_clean.service -P "${HOME}/.config/systemd/user"
wget https://raw.githubusercontent.com/NUbots/NUbots/master/doc/BuildKiteDockerClean/docker_clean.timer -P "${HOME}/.config/systemd/user"
```

2. Download cleaning script into home folder
```
wget https://raw.githubusercontent.com/NUbots/NUbots/master/doc/BuildKiteDockerClean/docker_clean.sh -P "${HOME}"
```

3. Make sure systemd knows there are new timer/service files
```
sudo systemctl daemon-reload
```

4. Enable and start the systemd timer
```
systemctl --user enable docker_clean.timer
systemctl --user start docker_clean.timer
```
