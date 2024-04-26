#! /bin/bash
##
## MIT License
##
## Copyright (c) 2019 NUbots
##
## This file is part of the NUbots codebase.
## See https://github.com/NUbots/NUbots for further info.
##
## Permission is hereby granted, free of charge, to any person obtaining a copy
## of this software and associated documentation files (the "Software"), to deal
## in the Software without restriction, including without limitation the rights
## to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
## copies of the Software, and to permit persons to whom the Software is
## furnished to do so, subject to the following conditions:
##
## The above copyright notice and this permission notice shall be included in all
## copies or substantial portions of the Software.
##
## THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
## IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
## FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
## AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
## LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
## OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
## SOFTWARE.
##


# Ensure that the time is accurate
timedatectl set-ntp true

################
# PARTITIONING #
################
DRIVE="/dev/nvme0n1"
BOOT="${DRIVE}p1"
ROOT="${DRIVE}p2"

# Setup partitions
parted -s ${DRIVE} mklabel gpt
parted -s ${DRIVE} mkpart primary fat32 1MiB 261MiB
parted -s ${DRIVE} set 1 esp on
parted -s ${DRIVE} mkpart primary ext4 261MiB 100%

# Format partitions
mkfs.fat ${BOOT}
mkfs.ext4 -F ${ROOT}

#################
# BOOTSTRAPPING #
#################

# Mount the partitions
mount ${ROOT} /mnt
mkdir -p /mnt/boot/efi
mount ${BOOT} /mnt/boot/efi

# Use pacman configuration from dockerfile
wget https://raw.githubusercontent.com/NUbots/NUbots/main/docker/etc/pacman.conf \
    -O /etc/pacman.conf
wget https://raw.githubusercontent.com/NUbots/NUbots/main/docker/etc/pacman.d/mirrorlist \
    -O /etc/pacman.d/mirrorlist
# Bootstrap Pacman and copy pacman config files to new root location
pacstrap /mnt base linux linux-firmware
cp /etc/pacman.conf /mnt/etc/pacman.conf
cp /etc/pacman.d/mirrorlist /mnt/etc/pacman.d/mirrorlist

# Update fstab
genfstab -U /mnt >> /mnt/etc/fstab

# Download install script into chroot drive
curl -L https://raw.githubusercontent.com/NUbots/NUbots/main/doc/ArchInstall/arch-chroot_install.sh \
    -o /mnt/arch-chroot_install.sh
chmod +x /mnt/arch-chroot_install.sh

echo ""
echo "####################################################################################"
echo "# Time to configure the new system. Run the following commands to achieve glory!!! #"
echo "# ROBOT_NUMBER=4 arch-chroot /mnt ./arch-chroot_install.sh                         #"
echo "####################################################################################"
