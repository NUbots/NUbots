#! /bin/bash

# Ensure that the time is accurate
timedatectl set-ntp true

################
# PARTITIONING #
################
DRIVE="/dev/nvme0n1"
BOOT="${DRIVE}p1"
ROOT="${DRIVE}p2"

# Remove all existing partitions
for v_partition in $(parted -s ${DRIVE} print|awk '/^ / {print $1}')
do
    parted -s ${DRIVE} rm ${v_partition}
done

# Setup partitions
parted -s ${DRIVE} mklabel gpt
parted -s ${DRIVE} mkpart primary fat32 1MiB 261MiB
parted -s ${DRIVE} mkpart primary ext4 261MiB 100%

# Format partitions
mkfs.fat ${BOOT}
mkfs.ext4 ${ROOT}

#################
# BOOTSTRAPPING #
#################

# Mount the partitions
mount ${ROOT} /mnt
mkdir -p /mnt/boot/efi
mount ${BOOT} /mnt/boot/efi

# Bootstrap Pacman
pacstrap /mnt base

# Update fstab
genfstab -U /mnt >> /mnt/etc/fstab

# Download install script into chroot drive
wget https://raw.githubusercontent.com/NUbots/NUbots/master/doc/ArchInstall/arch-chroot_install.sh \
    -O /mnt/arch-chroot_install.sh
chmod +x /mnt/arch-chroot_install.sh

echo ""
echo "####################################################################################"
echo "# Time to configure the new system. Run the following commands to achieve glory!!! #"
echo "# ROBOT_NUMBER=4 arch-chroot /mnt ./arch-chroot_install.sh                         #"
echo "####################################################################################"
