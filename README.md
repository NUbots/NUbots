nubots-puppet
=============

Configuration management for the NUbots team.

# Directions for setting up a NUbots development VM:

1.	Install Ubuntu 12.04 on a new VirtualBox VM:
	(currently using `ubuntu-12.04.3-desktop-i386.iso` downloaded from [here](http://releases.ubuntu.com/precise/ubuntu-12.04.3-desktop-i386.iso)).

	Create VM:
	* Name: Nubots-NUClearPort (or whatever)
	* Operating System: Ubuntu
	* Base Memory: 4096 MB (use at least 1024 MB for reasonable performance)
	* Hard Disk: Create New + Fixed-Size + 6.00 GB

	Open Settings:
	* General -> Advanced -> Shared Clipboard: Bidirectional
	* General -> Advanced -> Drag 'n' Drop: Bidirectional
	* System -> Motherboard -> Enable IO APIC: Enable
	* System -> Processor -> Processors: 4 (at least 1/4 to 1/2 of your CPUs)
	* Network -> Adapter 1 -> Enable Network Adapter: Enable
	* Network -> Adapter 1 -> Attached To: Bridged Adapter
	* Shared Folders -> Add: Name 'nubots', Path '/Users/mitchell/nubots'. (for example, on OSX)

	Start the machine: (Select `ubuntu-12.04.3-desktop-i386.iso` as the installation media)
	
		1. Welcome: Install Ubuntu
		2. Download updates while installing.
		3. Erase disk and install Ubuntu
		4. Install now
		5. Manual partition
		6. Delete both partitions + add 4000 MB of ext4 with mount '/', and the rest as swap.
		7. Install now
		8. Set up location + language + keyboard + user + password however you want!
			Note: NUbots' go-to settings are:
			* Sydney
			* US English
			* US English
			* nubot
			* 123123 (+ don't require password to login)
	
	Install VirtualBox Guest Additions:
	* On the host:
		Devices -> Install Guest Additions...
	* On the guest:
		When presented with the message:
			`This medium contains software intended to be automatically started. Would you like to run it?`,
		Select `Run`.
	
		Alternatively:
			
			$ sudo  '/media/VBOXADDITOINS_4.2.18_88780/VBoxLinuxAdditions.run'
	* Restart the VM.

__(Note: commands in all following steps must be run on the new VM)__

2. Enable the puppetlabs repository for Ubuntu 12.04 Precise Pangolin:

		$ wget http://apt.puppetlabs.com/puppetlabs-release-precise.deb
		$ sudo dpkg -i puppetlabs-release-precise.deb
		$ sudo apt-get update

3. Install Puppet and Git:

		$ sudo apt-get install puppet
		$ sudo apt-get install git

4. Clone the git repo into `/etc/puppet`:

		$ sudo git clone https://github.com/mitchellmetcalfe/nubots-puppet.git /etc/puppet

	Important: Make sure to set your git identiy correctly.
	
		$ git config --global user.name "Your Name"
		$ git config --global user.email you@example.com

		$ git config --global color.ui auto

5. Make Puppet apply its configuration to the VM:

		$ sudo puppet apply /etc/puppet/manifests/site.pp

6. Make robots do awesome stuff!
