#! /bin/bash

# Prepare useful variables
USER="nubots"
HOME="/home/${USER}"

HOST="nugus"
HOSTNAME="${HOST}${ROBOT_NUMBER}"
IP_ADDR="10.1.1.${ROBOT_NUMBER}"
ETHERNET_INTERFACE="eno1"
WIFI_INTERFACE=$(udevadm test-builtin net_id /sys/class/net/wlan0 2>/dev/null | grep ID_NET_NAME_PATH | cut -d = -f2)

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
sed --in-place 's/^#\s*\(%wheel\s\+ALL=(ALL)\s\+NOPASSWD:\s\+ALL\)/\1/' /etc/sudoers

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
	vim \
	nano \
	wget \
	screen \
	htop \
	gdb \
	linux-headers \
	bluez \
	bluez-utils \
	rsync

##############
# NETWORKING #
##############

# Enable the ssh server
systemctl enable sshd.service

# Setup the ssh issue file
mkdir banner
wget https://raw.githubusercontent.com/NUbots/NUbots/master/nuclear/roles/banner/bigtext.py -O bigtext.py
wget https://raw.githubusercontent.com/NUbots/NUbots/master/cmake/banner.png -O banner.png
wget https://raw.githubusercontent.com/NUbots/NUbots/master/doc/ArchInstall/motd -O /etc/motd
cat << EOF > generate_banner.py
import bigtext

with open("/etc/motd", "a") as f:
    f.write(bigtext.bigtext("NUgus ${ROBOT_NUMBER}"))

EOF
python ./generate_banner.py
rm -rf banner.png bigtext.py generate_banner.py

# Setup the fallback ethernet static connection
cat << EOF > /etc/systemd/network/99-${ETHERNET_INTERFACE}-static.network
[Match]
Name=${ETHERNET_INTERFACE}

[Network]
Address=${IP_ADDR}/16
Gateway=10.1.3.1
DNS=10.1.3.1
DNS=8.8.8.8
EOF

# Setup the fallback wireless static connection
cat << EOF > /etc/systemd/network/99-${WIFI_INTERFACE}-static.network
[Match]
Name=${WIFI_INTERFACE}

[Network]
Address=${IP_ADDR}/16
Gateway=10.1.3.1
DNS=10.1.3.1
DNS=8.8.8.8
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

# Create the bond interface
cat << EOF > /etc/systemd/network/10-bond0.netdev
[NetDev]
Name=bond0
Kind=bond

[Bond]
Mode=active-backup
MIIMonitorSec=1s
EOF

# Configure the bond
cat << EOF > /etc/systemd/network/10-bond0.network
[Match]
Name=bond0

[Network]
Address=${IP_ADDR}/16
Gateway=10.1.3.1
DNS=10.1.3.1
DNS=8.8.8.8
EOF

# Setup the bond ethernet connection
cat << EOF > /etc/systemd/network/20-${ETHERNET_INTERFACE}.network
[Match]
Name=${ETHERNET_INTERFACE}

[Network]
Bond=bond0
PrimarySlave=true
EOF

# Setup the bond wireless connection
cat << EOF > /etc/systemd/network/30-${WIFI_INTERFACE}.network
[Match]
Name=${WIFI_INTERFACE}

[Network]
Bond=bond0
EOF

# Enable and start both networking and dns services
systemctl enable systemd-networkd.service
systemctl enable systemd-resolved.service
systemctl enable wpa_supplicant
systemctl enable wpa_supplicant@${WIFI_INTERFACE}

# Populate udev rules.
cat << EOF > /etc/udev/rules.d/10-nubots.rules
# Set permissions for ttyUSB0 (CM740) and video* (webcam) devices
KERNEL=="ttyUSB*", MODE="0666"
KERNEL=="video*", MODE="0666"

# Symlink the CM740 device to /dev/CM740
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", SYMLINK+="CM740"

# Make sure FLIR cameras end up in the u3v group
SUBSYSTEM=="usb", ATTRS{idVendor}=="1e10", GROUP="u3v"
EOF

# Install robocup.service to allow robocup to autostart
cat << EOF > /etc/systemd/system/robocup.service
[Unit]
Description="RoboCup auto-start unit"
Wants=network.target
After=network.target
RequiresMountsFor=${HOME}

[Service]
Type=simple
Restart=always
WorkingDirectory=${HOME}
User=${USER}
Environment=HOME="${HOME}/"
PassEnvironment=HOME
ExecStart=${HOME}/robocup

[Install]
WantedBy=multi-user.target
EOF

# Make sure the system checks /usr/local for libraries
echo -e "/usr/local/lib\n/usr/local/lib64" > /etc/ld.so.conf.d/usrlocal.conf
ldconfig

# Make sure python checks /usr/local for packages
echo $(python -c "import site; print(site.getsitepackages()[0].replace('/usr', '/usr/local'))") \
    > $(python -c "import site; print(site.getsitepackages()[0])")/local.pth
wget https://raw.githubusercontent.com/NUbots/NUbots/master/docker/etc/pip.conf -O /etc/pip.conf

#############
# ZSH SHELL #
#############

# Install zsh and git
pacman -S --noconfirm --needed zsh zsh-completions git

# Download zprezto to the user home directory
cd ${HOME}
git clone --recursive https://github.com/sorin-ionescu/prezto.git .zprezto
for f in .zprezto/runcoms/z*; do
    ln -s $f .$(basename $f)
done

# Change user shell to zsh
chsh -s /usr/bin/zsh ${USER}

# Get fuzzy find and install it
pacman -S --noconfirm --needed fzf
cat << EOF >> ${HOME}/.zshrc

# Source the fuzzy find scripts
source /usr/share/fzf/key-bindings.zsh
source /usr/share/fzf/completion.zsh
EOF

############
# SSH KEYS #
############

# Create user ssh directory
mkdir -p ${HOME}/.ssh

# Register docker client as an authorized user
wget https://raw.githubusercontent.com/NUbots/NUbots/master/docker/home/nubots/.ssh/id_rsa.pub \
    -O ${HOME}/.ssh/authorized_keys

# Fix permissions so ssh will accept our authorized keys
chmod 600 ${HOME}/.ssh/authorized_keys

##########
# OpenCL #
##########

# Install the OpenCL icd file
mkdir -p /etc/OpenCL/vendors
wget https://raw.githubusercontent.com/NUbots/NUbots/master/docker/etc/OpenCL/vendors/intel.icd \
    -O /etc/OpenCL/vendors/intel.icd

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
wget https://raw.githubusercontent.com/NUbots/NUbots/master/doc/ArchInstall/arch-post_install.sh \
    -O /arch-post_install.sh
chmod +x /arch-post_install.sh

echo ""
echo "#####################################################################################"
echo "# New system is now configured. Run the following command then unplug the USB drive #"
echo "# /mnt/arch-post_install.sh                                                         #"
echo "#####################################################################################"
