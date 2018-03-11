# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure("2") do |config|
  host = RbConfig::CONFIG['host_os']

  if host =~ /darwin/
    cpus = `sysctl -n hw.physicalcpu_max`.to_i
    # sysctl returns Bytes, convert to KB
    memory = `sysctl -n hw.memsize`.to_i / 1024
  elsif host =~ /linux/
    cpus = `lscpu -p | egrep -v '^#' | sort -u -t, -k 2,4 | wc -l`.to_i
    # meminfo returns KB already
    memory = `grep 'MemTotal' /proc/meminfo | sed -e 's/MemTotal://' -e 's/ kB//'`.to_i
  elsif host =~ /mswin|mingw|cygwin/
    cpus = `wmic cpu get NumberOfCores`.split[1].to_i
    # Get TotalPhysicalMemory returns Bytes, convert to KB
    memory = `wmic computersystem Get TotalPhysicalMemory`.split[1].to_i / 1024
  end

  # Convert memory to MB for for Vagrant
  memory = memory / 1024

  # Use half the system memory for VM
  memory = memory / 2

  # Settings for a parallels provider
  config.vm.provider "parallels" do |v, override|
    # Use parallels virtualbox
    if ENV['TRAVISVM']
        override.vm.box = "parallels/ubuntu-14.04"
    else
        override.vm.box = "parallels/ubuntu-16.04"
    end

    # See http://www.virtualbox.org/manual/ch08.html#vboxmanage-modifyvm
    # and http://parallels.github.io/vagrant-parallels/docs/configuration.html
    v.customize ["set", :id, "--cpus", cpus ]
    v.customize ["set", :id, "--memsize", memory ]
    v.update_guest_tools = true
  end

  # Settings if using a virtualbox provider
  config.vm.provider "virtualbox" do |v, override|
    # Use custom box because official Ubuntu one is shit.
    if ENV['TRAVISVM']
        override.vm.box = "ubuntu/trusty64"
    else
        override.vm.box = "bidski/xenial64"
    end

    override.vm.boot_timeout = 360

    # See http://www.virtualbox.org/manual/ch08.html#vboxmanage-modifyvm
    v.customize ["modifyvm", :id, "--cpus", cpus ]
    v.customize ["modifyvm", :id, "--memory", memory ]
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

  config.vm.provision "add-32bit", type: "shell" do |shell|
      shell.inline = "dpkg --add-architecture i386 && apt-get update"
  end

  # Before anything else runs make sure dpkg isnt locked.
  # Might as well do a quick update while we are here
  config.vm.provision "unlock-dpkg", type: "shell", run: "always" do |shell|
    shell.inline = "rm /var/lib/dpkg/lock;
                    apt-get update;
                    apt-get dist-upgrade -y;
                    apt-get autoremove --purge -y;"
  end

  # Before the puppet provisioner runs
  # install puppet modules that are used
  config.vm.provision "install-puppet-modules", type: "shell" do |shell|
    shell.inline = "codename=$(lsb_release -sc) && \\
                    if [ \"${codename}\" == \"trusty\" ]; then \\
                        wget -N https://apt.puppetlabs.com/puppetlabs-release-trusty.deb && \\
                        dpkg -i puppetlabs-release-trusty.deb && \\
                        apt-get update; \\
                    fi; \\
                    apt-get install -y --reinstall puppet; \\
                    mkdir -p /etc/puppet/modules; \\
                    puppet module list | grep -q 'puppetlabs-apt' \\
                         || puppet module install puppetlabs-apt --module_repository https://forge.puppet.com --version 2.4.0;
                    puppet module list | grep -q 'puppetlabs-vcsrepo' \\
                         || puppet module install puppetlabs-vcsrepo --module_repository https://forge.puppet.com;
                    puppet module list | grep -q 'camptocamp-archive' \\
                         || puppet module install camptocamp-archive --module_repository https://forge.puppet.com;
                    puppet module list | grep -q 'maestrodev-wget' \\
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

  # Define the NUbots development VM, and make it the primary VM
  # (meaning that a plain `vagrant up` will only create this machine)
  # This VM will install all dependencies using the NUbots deb file (faster, generally recommended)
  config.vm.define "travisvm", autostart: false, primary: false do |nubots|
    nubots.vm.hostname = "nubotsvmbuild.nubots.net"

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
