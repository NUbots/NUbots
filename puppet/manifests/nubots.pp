include apt
include archive::prerequisites
include installer::prerequisites

# http://www.puppetcookbook.com/posts/set-global-exec-path.html
Exec { path => [ "/bin/", "/sbin/" , "/usr/bin/", "/usr/sbin/" ] }

node nubotsvm {

  # We need dev tools
  class {'dev_tools': }

  $toolchain_url = "https://drive.google.com/file/d/0B-0Hu8H3MvexUnFaX1FXc21wM3M/view?usp=sharing"

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
