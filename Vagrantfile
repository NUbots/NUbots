# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure("2") do |config|

  # Settings for a parallels provider
  config.vm.provider "parallels" do |v, override|
    # Use parallels virtualbox
    override.vm.box = "parallels/ubuntu-14.04-i386"

    # See http://www.virtualbox.org/manual/ch08.html#vboxmanage-modifyvm
    # and http://parallels.github.io/vagrant-parallels/docs/configuration.html
    v.memory = 8192
    v.cpus = 4
    v.update_guest_tools = true
  end

  # Settings if using a virtualbox provider
  config.vm.provider "virtualbox" do |v, override|
    # Use the official ubuntu box
    override.vm.box = "ubuntu/trusty32"

    # See http://www.virtualbox.org/manual/ch08.html#vboxmanage-modifyvm
    v.memory = 4096
    v.cpus = 4
    v.customize ["modifyvm", :id, "--vram", 128]
    v.customize ["modifyvm", :id, "--ioapic", "on"]
    v.customize ["modifyvm", :id, "--accelerate3d", "on"]
  end

  # Use Ubuntu 14.04 32bit VM
  config.vm.box = "puphpet/ubuntu1404-x32"

  # Fix the no tty error when installing
  config.vm.provision "fix-no-tty", type: "shell" do |shell|
    shell.privileged = false
    shell.inline = "sudo sed -i '/tty/!s/mesg n/tty -s \\&\\& mesg n/' /root/.profile"
  end

  # Before the puppet provisioner runs
  # install puppet modules that are used
  config.vm.provision "install-puppet-modules", type: "shell" do |shell|
    shell.inline = "apt-get install -y puppet;
                    mkdir -p /etc/puppet/modules;
                    puppet module list | grep -q 'puppetlabs-apt' \
                         || puppet module install puppetlabs-apt;
                    puppet module list | grep -q 'puppetlabs-vcsrepo' \
                         || puppet module install puppetlabs-vcsrepo;
                    puppet module list | grep -q 'camptocamp-archive' \
                         || puppet module install camptocamp-archive;
                    puppet module list | grep -q 'maestrodev-wget' \
                         || puppet module install maestrodev-wget;"
  end

  # Enable provisioning with Puppet stand alone.  Puppet manifests
  # are contained in a directory path relative to this Vagrantfile.
  config.vm.provision :puppet do |puppet|
    puppet.manifests_path = "puppet/manifests"
    puppet.module_path = "puppet/modules"
    puppet.manifest_file = "nubots.pp"
    puppet.options = [
      # See https://docs.puppetlabs.com/references/3.6.2/man/agent.html#OPTIONS
      "--verbose",
      "--debug"
    ]
  end


  # Define the NUbots development VM, and make it the primary VM
  # (meaning that a plain `vagrant up` will only create this machine)
  # This VM will install all dependencies using the NUbots deb file (faster, generally recommended)
  config.vm.define "nubotsvm", primary: true do |nubots|
    nubots.vm.hostname = "nubotsvm.nubots.net"

    # Note: Use NFS for more predictable shared folder support.
    #   The guest must have 'apt-get install nfs-common'
    nubots.vm.synced_folder ".", "/home/vagrant/NUbots"

    # Private network for NUsight's benifit
    nubots.vm.network "public_network", type: "dhcp"

    # Share NUsight repository with the VM if it has been placed in the same
    # directory as the NUbots repository
    if File.directory?("../NUsight")
      nubots.vm.synced_folder "../NUsight", "/home/vagrant/NUsight"
    end
    if File.directory?("../NUClear")
      nubots.vm.synced_folder "../NUClear", "/home/vagrant/NUClear"
    end
  end

  # This VM will build all dependencies by source (use this to update old dependencies, or to generate a new deb file)
  config.vm.define "nubotsvmbuild", autostart: false do |nubots|
    nubots.vm.hostname = "nubotsvmbuild.nubots.net"

    # Note: Use NFS for more predictable shared folder support.
    #   The guest must have 'apt-get install nfs-common'
    nubots.vm.synced_folder ".", "/home/vagrant/NUbots"

    # Share NUsight repository with the VM if it has been placed in the same
    # directory as the NUbots repository
    if File.directory?("../NUsight")
      nubots.vm.synced_folder "../NUsight", "/home/vagrant/NUsight"
    end
    if File.directory?("../NUClear")
      nubots.vm.synced_folder "../NUClear", "/home/vagrant/NUClear"
    end
  end
end
