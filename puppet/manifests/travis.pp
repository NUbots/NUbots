include apt

# http://www.puppetcookbook.com/posts/set-global-exec-path.html
Exec { path => [ "/bin/", "/sbin/" , "/usr/bin/", "/usr/sbin/" ] }

node default {

  # We need build tools
  class {'build_tools': }

  # Get and install our toolchain
  $toolchain_version = '2.1.1'

  file { '/nubots':
    ensure => 'directory',
  } ->
  wget::fetch { 'nubots_deb':
    destination => "/nubots/nubots-toolchain-${toolchain_version}.deb",
    source      => "http://nubots.net/debs/nubots-toolchain-${toolchain_version}.deb",
    timeout     => 0,
  } ->
  package { 'nubots-toolchain':
    provider => 'dpkg',
    ensure   => 'latest',
    source   => "/nubots/nubots-toolchain-${toolchain_version}.deb",
  }
}
