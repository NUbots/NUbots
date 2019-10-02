#! /bin/bash

# Unable to make this symlink in the chroot environment as /etc/resolv.conf is a mounted volume
ln -sf /run/systemd/resolve/stub-resolv.conf /mnt/etc/resolv.conf

# Unmount the partitions
umount /mnt/boot/efi
umount /mnt

# Now restart the system
reboot
