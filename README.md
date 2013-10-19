nubots-puppet
=============

Configuration management for the NUbots team.

# Directions for setting up a NUbots development VM:

1. Install Ubuntu 12.04 on a new VirtualBox VM:
	(currently using ubuntu-12.04.3-desktop-i386.iso downloaded from [here](http://releases.ubuntu.com/precise/ubuntu-12.04.3-desktop-i386.iso)).

2. Install VirtualBox Guest Additions:
	On the host:
		- Devices -> Install Guest Additions...
	On the guest:
		$ sudo  '/media/VBOXADDITOINS_4.2.18_88780/VBoxLinuxAdditions.run'
		OR just let the autorun script go.

	Then restart the VM.

3. Enable the puppetlabs repository for Ubuntu 12.04 Precise Pangolin:
	$ wget http://apt.puppetlabs.com/puppetlabs-release-precise.deb
	$ sudo dpkg -i puppetlabs-release-precise.deb
	$ sudo apt-get update

4. Install Puppet and Git:
	$ sudo apt-get install puppet
	$ sudo apt-get install git

5. Clone the git repo into /etc/puppet
	$ sudo git clone https://github.com/mitchellmetcalfe/nubots-puppet.git /etc/puppet

	Important: Make sure to set your git identiy correctly.
		$ git config --global user.name "Your Name"
		$ git config --global user.email you@example.com

		$ git config --global color.ui auto

6. Make Puppet apply its configuration to the VM:
	$ sudo puppet apply /etc/puppet/manifests/site.pp

7. Make robots do awesome stuff!
