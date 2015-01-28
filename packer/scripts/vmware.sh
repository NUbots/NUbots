apt-get -y install dkms

# Mount the disk image:
mkdir /mnt/vmware-tools-iso
mount -o loop linux.iso /mnt/vmware-tools-iso

# Store the current directory (can't use pushd/popd in sh):
currentdir=$(pwd)

# Get the name of the tools archive:
# e.g. VMwareTools-9.6.1-1378637.tar.gz
cd /mnt/vmware-tools-iso/
toolsname=$(ls VMwareTools-*.gz)

# Install the tools:
cp /mnt/vmware-tools-iso/$toolsname /tmp/

cd /tmp

tar -zxvf $toolsname

./vmware-tools-distrib/vmware-install.pl --default

# Restore the current directory:
cd $currentdir

# Clean up:
umount /mnt/vmware-tools-iso

rm -rf /mnt/vmware-tools-iso \
       linux.iso             \
       /tmp/$toolsname       \
       /tmp/vmware-tools-distrib


# Fix "Waiting for HGFS kernel module to load..." timeout bug.
# See: http://dantehranian.wordpress.com/2014/08/19/vagrant-vmware-resolving-waiting-for-hgfs-kernel-module-timeouts/
echo "answer AUTO_KMODS_ENABLED yes" | sudo tee -a /etc/vmware-tools/locations
