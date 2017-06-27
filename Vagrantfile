# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure("2") do |config|

  # Settings for a parallels provider
  config.vm.provider "parallels" do |v, override|
    # Use parallels virtualbox
    override.vm.box = "parallels/ubuntu-16.04"

    # See http://www.virtualbox.org/manual/ch08.html#vboxmanage-modifyvm
    # and http://parallels.github.io/vagrant-parallels/docs/configuration.html
    v.customize ["set", :id, "--cpus", `sysctl -n hw.physicalcpu_max 2> /dev/null`.chomp ]
    v.customize ["set", :id, "--memsize", `echo "scale=0; $(sysctl -n hw.memsize 2> /dev/null || echo 0)/2097152" | bc`.chomp ]
    v.update_guest_tools = true
  end

  # Settings if using a virtualbox provider
  config.vm.provider "virtualbox" do |v, override|
    # Use the official ubuntu box
    #override.vm.box = "ubuntu/xenial64"

    # Use custom box because official Ubuntu one is shit.
    override.vm.box = "bidski/xenial64"

    # See http://www.virtualbox.org/manual/ch08.html#vboxmanage-modifyvm
    v.customize ["modifyvm", :id, "--cpus", `if [ "x$(uname)" = "xDarwin" ]; then sysctl -n hw.physicalcpu_max; else lscpu -p | egrep -v '^#' | sort -u -t, -k 2,4 | wc -l; fi`.chomp ]
    v.customize ["modifyvm", :id, "--memory", `if [ "x$(uname)" = "xDarwin" ]; then echo "scale=0; $(sysctl -n hw.memsize)/2097152" | bc; else echo "scale=0; $(awk '/MemTotal/{print $2}' /proc/meminfo)/2048" | bc; fi`.chomp ]
    v.customize ["modifyvm", :id, "--vram", 128]
    v.customize ["modifyvm", :id, "--ioapic", "on"]
    v.customize ["modifyvm", :id, "--accelerate3d", "on"]
    v.customize ["modifyvm", :id, "--cableconnected1", "on"]
    v.customize ["modifyvm", :id, "--natdnshostresolver1", "on"]
  end

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
                         || puppet module install puppetlabs-apt --module_repository https://forge.puppet.com --version 2.4.0;
                    puppet module list | grep -q 'puppetlabs-vcsrepo' \
                         || puppet module install puppetlabs-vcsrepo --module_repository https://forge.puppet.com;
                    puppet module list | grep -q 'camptocamp-archive' \
                         || puppet module install camptocamp-archive --module_repository https://forge.puppet.com;
                    puppet module list | grep -q 'maestrodev-wget' \
                         || puppet module install maestrodev-wget --module_repository https://forge.puppet.com;"
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
      "--debug",
      "--parser=future"
    ]
  end


  # Define the NUbots development VM, and make it the primary VM
  # (meaning that a plain `vagrant up` will only create this machine)
  # This VM will install all dependencies using the NUbots deb file (faster, generally recommended)
  config.vm.define "nubotsvm", autostart: true, primary: true do |nubots|
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
    if File.directory?("../CM730")
      nubots.vm.synced_folder "../CM730", "/home/vagrant/CM730"
    end
  end

  # This VM will build all dependencies by source (use this to update old dependencies, or to generate a new deb file)
  config.vm.define "nubotsvmbuild", autostart: false, primary: false do |nubots|
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
    if File.directory?("../CM730")
      nubots.vm.synced_folder "../CM730", "/home/vagrant/CM730"
    end
  end
end
