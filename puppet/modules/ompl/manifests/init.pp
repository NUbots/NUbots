# == Class: ompl
class ompl {

# Note: This installs an ompl .deb that was manually created using the
# following process:
/*
mkdir -p /tmp/omplinstall
cd /tmp/omplinstall
sudo apt-get install libeigen3-dev
sudo apt-get install libboost-system-dev
sudo apt-get install libboost-filesystem-dev
sudo apt-get install libboost-thread-dev
sudo apt-get install libboost-serialization-dev
sudo apt-get install libboost-program-options-dev
sudo apt-get install libboost-test-dev
sudo apt-get install libboost-chrono-dev
sudo apt-get install libboost-date-time-dev
sudo apt-get install libffi-dev
sudo apt-get install ruby-dev
sudo gem install fpm
git clone https://github.com/ompl/ompl.git
cd ompl
cd build
mkdir build
cmake .. -G Ninja
ninja
mkdir /tmp/omplinstall/debdir
DESTDIR=/tmp/omplinstall/debdir ninja install
fpm -s dir -t deb -n ompl -v 0.14.2 \
                          -C /tmp/omplinstall/debdir \
                          -d libboost-system-dev \
                          -d libboost-filesystem-dev \
                          -d libboost-thread-dev \
                          -d libboost-serialization-dev \
                          -d libboost-program-options-dev \
                          -d libboost-test-dev \
                          -d libboost-chrono-dev \
                          -d libboost-date-time-dev \
                          -d libeigen3-dev \
                          usr/local/bin/ usr/local/include usr/local/lib usr/local/share
*/
  $ompl_pkg = 'ompl_0.14.2_i386.deb'

  wget::fetch { "${ompl_pkg}":
    require    => [
      Package['libeigen3-dev'],
      Package['libboost-system-dev'],
      Package['libboost-filesystem-dev'],
      Package['libboost-thread-dev'],
      Package['libboost-serialization-dev'],
      Package['libboost-program-options-dev'],
      Package['libboost-test-dev'],
      Package['libboost-chrono-dev'],
      Package['libboost-date-time-dev'],
    ],
    destination => "/tmp/${ompl_pkg}",
    source      => "https://dl.dropboxusercontent.com/u/27568639/development-files/${ompl_pkg}",
  }
  ~> package { 'ompl':
    provider => 'dpkg',
    ensure => 'latest',
    source => "/tmp/${ompl_pkg}"
  }

  # file { 'copy_ompl_install':
  #   require => [
  #     Package['libeigen3-dev'],
  #     Package['libboost-system-dev'],
  #     Package['libboost-filesystem-dev'],
  #     Package['libboost-thread-dev'],
  #     Package['libboost-serialization-dev'],
  #     Package['libboost-program-options-dev'],
  #     Package['libboost-test-dev'],
  #     Package['libboost-chrono-dev'],
  #     Package['libboost-date-time-dev'],
  #   ],
  #   ensure => 'file',
  #   source => 'puppet:///modules/ompl/ompl-install.sh',
  #   path => '/tmp/ompl-install.sh',
  #   # owner => 'root'
  #   # group => 'root'
  #   mode  => '0744',
  #   notify => Exec['run_ompl_install'],
  # }
  # exec { 'run_ompl_install':
  #   command => '/tmp/ompl-install.sh',
  #   timeout     => 1800,
  #   refreshonly => true,
  # }
}
