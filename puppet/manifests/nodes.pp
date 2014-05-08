class initial_apt_update {
  # Perform a single `apt-get update` before installing ANY packages.
  exec { "initial_apt_update":
    command => "/usr/bin/apt-get update"
  } -> Package <| |>
}

node nuclearportvm {
  include initial_apt_update

  # # define variables for this node
  $username = 'vagrant'

  class { 'nuclearport::build_dep': username => $username, }

  class { 'nubugger': username => $username, }

  # ps3 controller tools
  package { 'software-properties-common': ensure => latest, }

  # Non-essential developer tools:
  class { 'vim':  username => $username, }
  package { 'screen': ensure => latest, }
  package { 'htop': ensure => latest, }
  package { 'gdb': ensure => latest, }
  package { 'cmake-curses-gui': ensure => latest, }
  package { 'linux-headers-generic': ensure => latest, }
  package { 'dos2unix': ensure => latest, }
  package { 'rsync': ensure => latest, }
}

# node nubuggervm {
#   include initial_apt_update

#   # define variables for this node
#   $username = 'vagrant'

#   # these (and most packages) should be virtual resources
#   package { 'build-essential': ensure => latest }
#   package { 'git': ensure => latest }

#   file { 'nubots_dir':
#     path => "/home/${username}/nubots",
#     ensure => directory,
#     owner => $username,
#     group => $username,
#   }

#   class { 'nubugger': username => $username, }

#   class { 'vim': username => $username, }
# }

node packer-virtualbox-iso, packer-vmware-iso {
  include initial_apt_update

  $username = 'vagrant'

  # nuclear::build_dep
  include catch

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
  # package { 'libboost-math-dev': ensure => latest }

  # nubugger::build_dep
  package { 'pkg-config': ensure => latest, }
  package { 'uuid-dev': ensure => latest, }
  package { 'nodejs': ensure => latest, }
  package { 'npm': ensure => latest, }

  # Non-essential developer tools:
  class { 'vim':  username => $username, }
  package { 'screen': ensure => latest, }
  package { 'htop': ensure => latest, }
  package { 'gdb': ensure => latest, }
  package { 'cmake-curses-gui': ensure => latest, }
  package { 'linux-headers-generic': ensure => latest, }
  package { 'dos2unix': ensure => latest, }
  package { 'rsync': ensure => latest, }

  # NFS for better Vagrant shared folders
  package { 'nfs-common': ensure => latest, }
    
  # sharing fix, see http://superuser.com/questions/736024/cannot-share-host-directory-with-virtualbox-guest-mint-16-64-bit
  file { '/sbin/mount.vboxsf':
  	ensure => 'link',
  	target => '/usr/lib/i386-linux-gnu/VBoxGuestAdditions/mount.vboxsf'
  }
}
