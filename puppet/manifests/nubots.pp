include apt
include archive::prerequisites
include installer::prerequisites

# http://www.puppetcookbook.com/posts/set-global-exec-path.html
Exec { path => [ "/bin/", "/sbin/" , "/usr/bin/", "/usr/sbin/" ] }

node nubotsvm {

  package { 'libboost-system-dev': }
  package { 'libboost-filesystem-dev': }
  package { 'libboost-thread-dev': }
  package { 'libboost-serialization-dev': }
  package { 'libboost-program-options-dev': }
  package { 'libboost-test-dev': }
  package { 'libboost-chrono-dev': }
  package { 'libboost-date-time-dev': }

  # We need dev tools
  class {'dev_tools': }
  $toolchain_url = "https://www.dropbox.com/s/gj86fejcncri4ip/nubots-toolchain1.0.0.deb"

  # Get and install our toolchain
  wget::fetch { "nubots_toolchain":
    destination => "/tmp/nubots-toolchain.deb",
    source      => "${toolchain_url}",
  }
  ~> package { 'nubots-toolchain':
    provider => 'dpkg',
    ensure => 'latest',
    source => "/tmp/nubots-toolchain.deb",
    require => [Package['libboost-system-dev'],
                Package['libboost-filesystem-dev'],
                Package['libboost-thread-dev'],
                Package['libboost-serialization-dev'],
                Package['libboost-program-options-dev'],
                Package['libboost-test-dev'],
                Package['libboost-chrono-dev'],
                Package['libboost-date-time-dev']]
  }
}
