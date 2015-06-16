# http://www.puppetcookbook.com/posts/set-global-exec-path.html
Exec { path => [ "/bin/", "/sbin/" , "/usr/bin/", "/usr/sbin/" ] }

class initial_apt_update {
  # Perform a single `apt-get update` before installing ANY packages.
  exec { "initial_apt_update":
    command => "/usr/bin/apt-get update"
  } -> Package <| |>
}

class virtualbox_sharing_fix {
  # sharing fix, see http://superuser.com/questions/736024/cannot-share-host-directory-with-virtualbox-guest-mint-16-64-bit
  file { '/sbin/mount.vboxsf':
    ensure => 'link',
    target => '/usr/lib/i386-linux-gnu/VBoxGuestAdditions/mount.vboxsf'
  } -> Package <| |>
}

class developer_tools {
  include vm_ssh_keys
  class { 'vim':  username => $username, }
  package { 'screen': ensure => latest, }
  package { 'htop': ensure => latest, }
  package { 'gdb': ensure => latest, }
  package { 'cmake-curses-gui': ensure => latest, }
  package { 'linux-headers-generic': ensure => latest, }
  package { 'dos2unix': ensure => latest, }
  package { 'rsync': ensure => latest, }
  # package { 'curl': ensure => latest, }
}

node nubotsvm {
  include initial_apt_update
  include virtualbox_sharing_fix
  include archive::prerequisites

  # define variables for this node
  $username = 'vagrant'

  class { 'robocup::build_dep': username => $username, }
  class { 'cppformat':  }

  class { 'nusight': username => $username, }

  # ps3 controller tools
  package { 'software-properties-common': ensure => latest, }

  # Non-essential developer tools:
  include developer_tools
}

node packer-virtualbox-iso, packer-vmware-iso {
  include initial_apt_update
  include virtualbox_sharing_fix

  $username = 'vagrant'

  # # nuclear::build_dep
  # include catch

  # nuclearport::build_dep
  package { 'build-essential': ensure => latest }
  package { 'cmake': ensure => latest }
  package { 'git': ensure => latest }
  # package { 'openssh-server': ensure => latest }
  package { ['libprotobuf-dev', 'protobuf-compiler']: ensure => latest }
  package { 'libzmq3-dev': ensure => latest }
  package { 'libespeak-dev': ensure => latest }
  package { 'librtaudio-dev': ensure => latest }
  package { 'libncurses5-dev': ensure => latest }
  package { 'libjpeg-turbo8-dev': ensure => latest }
  package { 'libfftw3-dev': ensure => latest }
  package { 'libaubio-dev': ensure => latest }
  package { 'libsndfile-dev': ensure => latest }
  package { 'libtcmalloc-minimal4': ensure => latest }

  # nusight::build_dep
  package { 'pkg-config': ensure => latest, }
  package { 'uuid-dev': ensure => latest, }
  class { 'nodejs':
    version => 'v0.12.0',
  }
  -> package { 'nodejs-legacy': ensure => latest, }

  # Non-essential developer tools:
  include developer_tools

  # NFS for better Vagrant shared folders
  package { 'nfs-common': ensure => latest, }
}
