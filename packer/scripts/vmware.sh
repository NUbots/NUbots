apt-get -y install dkms

mkdir /mnt/vmware-tools-iso

mount -o loop linux.iso /mnt/vmware-tools-iso

cp /mnt/vmware-tools-iso/VMwareTools-9.6.1-1378637.tar.gz /tmp/

currentdir=pwd

cd /tmp

tar -zxvf VMwareTools-9.6.1-1378637.tar.gz

cd vmware-tools-distrib/

./vmware-install.pl --default

cd $currentdir

umount /mnt/vmware-tools-iso
