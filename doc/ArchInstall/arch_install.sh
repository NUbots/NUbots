#! /bin/bash

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

# Bootstrap Pacman and trust everything to prevent pgp key errors
cat << EOF > "/etc/pacman.conf"
[options]
HoldPkg      = pacman glibc
Architecture = auto
ParallelDownloads = 5
CheckSpace
Color

# Trust everything so we don't get signature errors for old packages while date locked
SigLevel = Never TrustAll

[core]
Include = /etc/pacman.d/mirrorlist

[extra]
Include = /etc/pacman.d/mirrorlist

[community]
Include = /etc/pacman.d/mirrorlist
EOF
pacstrap /mnt base linux linux-firmware
cp /etc/pacman.conf /mnt/etc/pacman.conf

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
