include apt
include archive::prerequisites
include installer::prerequisites

# http://www.puppetcookbook.com/posts/set-global-exec-path.html
Exec { path => [ "/bin/", "/sbin/" , "/usr/bin/", "/usr/sbin/" ] }

node nubotsvm {

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
    source => "/tmp/nubots-toolchain.deb"
  }

}
