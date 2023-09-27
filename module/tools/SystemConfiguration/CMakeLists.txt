# Docker embeds a customised mirrorlist into the built image. This mirrorlist directs Arch Linux/pacman to only pull
# packages from a specific date. Installing this mirrorlist on to the robots will help keep the robots in sync with the
# docker image.
nuclear_module(DATA_FILES "/etc/pacman.d/mirrorlist:system/default/etc/pacman.d" "/etc/pacman.conf:system/default/etc")
