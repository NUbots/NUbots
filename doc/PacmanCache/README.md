# Package cache for pacman packages

Sets up a package cache for caching [pacman](https://wiki.archlinux.org/index.php/pacman) packages locally.

## Prerequisites
 - [wget](https://www.gnu.org/software/wget/)
 - [docker-compose](https://docs.docker.com/compose/)

## Install

1. Create a directory do store cache config
```
mkdir ~/pacman-cache && cd ~/pacman-cache
```

2. Download install script
```
wget https://raw.githubusercontent.com/NUbots/NUbots/deKoeyer/pacman-cache/doc/PacmanCache/install.sh
```

3. Make install script executable
```
chmod +x ./install.sh
```

4. Run install script
```
./install.sh
```
