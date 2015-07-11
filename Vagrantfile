# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure("2") do |config|

  # Use Ubuntu 14.04 32bit VM
  config.vm.box = "puphpet/ubuntu1404-x32"

  # Create a public network, which generally matched to bridged network.
  # Bridged networks make the machine appear as another physical device on
  # your network.
  # config.vm.network :public_network

  # Enable provisioning with Puppet stand alone.  Puppet manifests
  # are contained in a directory path relative to this Vagrantfile.
  config.vm.provision :puppet do |puppet|
    puppet.manifests_path = "puppet/manifests"
    puppet.module_path = "puppet/modules"
    puppet.manifest_file  = "nubots.pp"
    puppet.options = [
      # See https://docs.puppetlabs.com/references/3.6.2/man/agent.html#OPTIONS
      "--verbose",
      "--debug"
    ]
  end

  config.vm.provider "virtualbox" do |v|
    # See http://www.virtualbox.org/manual/ch08.html#vboxmanage-modifyvm
    v.memory = 4096
    v.cpus = 4
    v.customize ["modifyvm", :id, "--vram", 128]
    v.customize ["modifyvm", :id, "--ioapic", "on"]
    v.customize ["modifyvm", :id, "--accelerate3d", "on"]
  end

  config.vm.provider "parallels" do |v|
    # See http://www.virtualbox.org/manual/ch08.html#vboxmanage-modifyvm
    # and http://parallels.github.io/vagrant-parallels/docs/configuration.html
    v.memory = 8192
    v.cpus = 4
    v.optimize_power_consumption = false
  end

  # Define the NUbots development VM, and make it the primary VM
  # (meaning that a plain `vagrant up` will only create this machine)
  # This VM will install all dependencies using the NUbots deb file (faster, generally recommended)
  config.vm.define "nubotsvm", primary: true do |nubots|
    nubots.vm.hostname = "nubotsvm.nubots.net"

    nubots.vm.network :private_network, ip: "192.168.33.77"

    # Uncomment this to enable a bridged adapter. This allows for the game controller running on your host to work with the virtual machine. Note: replace 'Wi-Fi' with your network adapter name.
    # See http://docs.vagrantup.com/v2/networking/public_network.html
    #config.vm.network "public_network", :bridge => 'Wi-Fi'

    nubots.vm.network :forwarded_port, guest: 12000, host: 12000
    nubots.vm.network :forwarded_port, guest: 12001, host: 12001

    # Add hostname here if running NUsight on the VM
    if [].include?(Socket.gethostname) # NUsight Port
      nubots.vm.network :forwarded_port, guest: 9090, host: 9090
    end

    # Note: Use NFS for more predictable shared folder support.
    #   The guest must have 'apt-get install nfs-common'
    nubots.vm.synced_folder ".", "/home/vagrant/nubots/NUbots"

    # Share NUsight repository with the VM if it has been placed in the same
    # directory as the NUbots repository
    if File.directory?("../NUsight")
      nubots.vm.synced_folder "../NUsight", "/home/vagrant/nubots/NUsight"
    end
  end

  # This VM will build all dependencies by source (use this to update old dependencies, or to generate a new deb file)
  config.vm.define "nubotsvmbuild", autostart: false do |nubots|
    nubots.vm.hostname = "nubotsvmbuild.nubots.net"

    # Note: Use NFS for more predictable shared folder support.
    #   The guest must have 'apt-get install nfs-common'
    nubots.vm.synced_folder ".", "/home/vagrant/nubots/NUbots"

    # Share NUsight repository with the VM if it has been placed in the same
    # directory as the NUbots repository
    if File.directory?("../NUsight")
      nubots.vm.synced_folder "../NUsight", "/home/vagrant/nubots/NUsight"
    end
  end
end
