# -*- mode: ruby -*-
# vi: set ft=ruby :

# Vagrantfile API/syntax version. Don't touch unless you know what you're doing!
VAGRANTFILE_API_VERSION = "2"

Vagrant.configure(VAGRANTFILE_API_VERSION) do |config|
  # Every Vagrant virtual environment requires a box to build off of.
  config.vm.box = "precise32"
  config.vm.box_url = "http://files.vagrantup.com/precise32.box"

  # Create a public network, which generally matched to bridged network.
  # Bridged networks make the machine appear as another physical device on
  # your network.
  # config.vm.network :public_network

  # Enable provisioning with Puppet stand alone.  Puppet manifests
  # are contained in a directory path relative to this Vagrantfile.
  config.vm.provision :puppet do |puppet|
    puppet.manifests_path = "puppet/manifests"
    puppet.module_path = "puppet/modules"
    puppet.manifest_file  = "site.pp"
  end

  # Define the NUClearPort development VM, snd make it the primary VM
  # (meaning that a plain `vagrant up` will only create this machine)
  config.vm.define "nuclearportvm", primary: true do |nuclearport|
    nuclearport.vm.hostname = "nuclearportvm"
    
    config.vm.network :private_network, ip: "192.168.33.77"

    # Syntax: "path/on/host", "/path/on/guest"
    nuclearport.vm.synced_folder ".", "/home/vagrant/nubots/NUClearPort"
  end

  # Define a VM for running NUbugger.
  # (use `vagrant up nubuggervm` to create it)
  config.vm.define "nubuggervm" do |nubugger|
    nubugger.vm.hostname = "nubuggervm"

    nubugger.vm.network :forwarded_port, guest: 9090, host: 9090
    nubugger.vm.network :forwarded_port, guest: 12000, host: 12000

    config.vm.network :private_network, ip: "192.168.33.88"

    # Share NUbugger repository with the VM if it has been placed in the same
    # directory as the NUClearPort repository
    if File.directory?("../NUbugger")
      nubugger.vm.synced_folder "../NUbugger", "/home/vagrant/nubots/NUbugger"
    end
  end
end
