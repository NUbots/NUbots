#! /bin/bash

# Prepare useful variables
USER="nubots"
HOME="/home/${USER}"

HOST="nugus"
HOSTNAME="${HOST}${ROBOT_NUMBER}"
IP_ADDR="10.1.1.${ROBOT_NUMBER}"
ETHERNET_INTERFACE=${ETHERNET_INTERFACE:-"enp86s0"}
WIFI_INTERFACE=${WIFI_INTERFACE:-"wlan0"}
WIFI_INTERFACE=$(udevadm test-builtin net_id /sys/class/net/${WIFI_INTERFACE} 2>/dev/null | grep ID_NET_NAME_PATH | cut -d = -f2)

# Setup timezone information
ln -sf /usr/share/zoneinfo/Australia/Sydney /etc/localtime
hwclock --systohc

# Setup locale
cat << EOF > /etc/locale.gen
en_AU.UTF-8 UTF-8
en_US.UTF-8 UTF-8
EOF
locale-gen

echo "LANG=en_AU.UTF-8" > /etc/locale.conf

# Setup hostname
echo "${HOSTNAME}" > /etc/hostname
cat << EOF > /etc/hosts
127.0.0.1       localhost
::1             localhost
127.0.1.1       ${HOSTNAME}
EOF

#########
# USERS #
#########

# Install sudo
pacman -S --noconfirm --needed sudo

# Setup users of the wheel group to be able to execute sudo commands with no password
echo "%wheel ALL=(ALL:ALL) NOPASSWD: ALL" > /etc/sudoers.d/wheel_sudo
chmod 440 /etc/sudoers.d/wheel_sudo

# Get sudo to insult users when they type a password wrong
echo "Defaults insults" > /etc/sudoers.d/insults

# Add u3v group and create our user
groupadd u3v
useradd -m -G wheel,lp,u3v ${USER}

# Lock the root account
passwd -l root

# Set the user password
echo "${USER}: " | chpasswd

##############
# BOOTLOADER #
##############

# Install the intel microcode
pacman -S --noconfirm --needed intel-ucode

# Install grub
pacman -S --noconfirm --needed grub efibootmgr
grub-install --target=x86_64-efi --efi-directory=/boot/efi --bootloader-id=GRUB --removable
sed 's/GRUB_TIMEOUT=.*/GRUB_TIMEOUT=0/' -i /etc/default/grub
sed 's/GRUB_TIMEOUT_STYLE=.*/GRUB_TIMEOUT_STYLE=hidden/' -i /etc/default/grub
grub-mkconfig -o /boot/grub/grub.cfg

# Update the system
pacman -Syu

# Install system utilities
pacman -S --noconfirm --needed \
	wpa_supplicant \
	openssh \
	wget \
	linux-headers \
	vim \
	nano \
	rsync \
	zsh

##############
# NETWORKING #
##############

# Enable the ssh server
systemctl enable sshd.service

# Setup a simple version of the MOTD file
cat << EOF > /etc/motd
NUgus ${ROBOT_NUMBER}
EOF

# Setup the fallback ethernet static connection
cat << EOF > /etc/systemd/network/99-ethernet-static.network
[Match]
Name=${ETHERNET_INTERFACE}

[Network]
Address=${IP_ADDR}/16
Gateway=10.1.3.1
DNS=10.1.3.1
DNS=8.8.8.8
EOF

# Setup the fallback wireless static connection
cat << EOF > /etc/systemd/network/99-wifi-static.network
[Match]
Name=${WIFI_INTERFACE}

[Network]
Address=${IP_ADDR}/16
Gateway=10.1.3.1
DNS=10.1.3.1
DNS=8.8.8.8
EOF

# Provide udevd configuration for network interfaces
cat << EOF > /etc/systemd/network/99-default.link
[Match]
OriginalName=*

[Link]
NamePolicy=path
MACAddressPolicy=persistent
EOF

# Setup wpa_supplicant networks
cat << EOF > /etc/wpa_supplicant/wpa_supplicant-${WIFI_INTERFACE}.conf
ctrl_interface=/var/run/wpa_supplicant
ctrl_interface_group=wheel
update_config=1
fast_reauth=1
ap_scan=1

network={
	ssid="epsilon-z"
	#psk="9181918191"
	psk=cf34016f6c385ae99246e5213cca8194f4b0aad498326ee1fb4f787271b84b8f
    priority=2
}
network={
	ssid="epsilon-x"
	#psk="9181918191"
	psk=cf34016f6c385ae99246e5213cca8194f4b0aad498326ee1fb4f787271b84b8f
    priority=1
}
EOF

# Enable and start both networking and dns services
systemctl enable systemd-networkd.service
systemctl enable systemd-resolved.service
systemctl enable wpa_supplicant
systemctl enable wpa_supplicant@${WIFI_INTERFACE}

#############
# ZSH SHELL #
#############

# Change user shell to zsh and create a blank config file to prevent user prompt
chsh -s /usr/bin/zsh ${USER}
touch ${HOME}/.zshrc

############
# SSH KEYS #
############

# Create user ssh directory
mkdir -p ${HOME}/.ssh

# Register docker client as an authorized user
wget https://raw.githubusercontent.com/NUbots/NUbots/main/docker/home/nubots/.ssh/id_rsa.pub \
    -O ${HOME}/.ssh/authorized_keys

# Fix permissions so ssh will accept our authorized keys
chmod 600 ${HOME}/.ssh/authorized_keys

#############
# LIBRARIES #
#############

# Use local user libraries so systemconfigurator can run
echo "/usr/local/lib" > /etc/ld.so.conf.d/usrlocal.conf

###############
# PERMISSIONS #
###############

# Fix all the permissions we just broke
chown -R ${USER}:${USER} ${HOME}

# Change ownership on /usr/local
chown -R ${USER}:${USER} /usr/local

################
# POST INSTALL #
################

# Download the post-install script into chroot drive
wget https://raw.githubusercontent.com/NUbots/NUbots/main/doc/ArchInstall/arch-post_install.sh \
    -O /arch-post_install.sh
chmod +x /arch-post_install.sh

echo ""
echo "#####################################################################################"
echo "# New system is now configured. Run the following command then unplug the USB drive #"
echo "# /mnt/arch-post_install.sh                                                         #"
echo "#####################################################################################"
