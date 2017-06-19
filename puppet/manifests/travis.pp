include apt

# http://www.puppetcookbook.com/posts/set-global-exec-path.html
Exec { path => [ "/bin/", "/sbin/" , "/usr/bin/", "/usr/sbin/" ] }

node default {

  # We need build tools
  class {'build_tools': }

  # Get and install our toolchain
  $toolchain_version = '2.1.1'
  file { 'nubots_deb':
    path   => "/root/nubots-toolchain-${toolchain_version}.deb",
    ensure => present,
    source => "http://nubots.net/debs/nubots-toolchain-${toolchain_version}.deb",
  } ->
  package { 'nubots-toolchain':
    provider => 'dpkg',
    ensure   => 'latest',
    source   => "/root/nubots-toolchain-${toolchain_version}.deb",
  }
}
