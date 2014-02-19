# Class: ruby::params
#
# This class handles the Ruby module parameters
#
class ruby::params {
  $version          = 'installed'
  $gems_version     = 'installed'
  $ruby_package     = 'ruby'
  $rubygems_package = 'rubygems'

  case $::osfamily {
    'redhat', 'amazon': {
      $ruby_dev = 'ruby-devel'
      $rubygems_update = true
    }
    'debian': {
      $ruby_dev = [ 'ruby-dev', 'rake', 'ri' ]
      $rubygems_update = false
    }
    default: {
      fail("Unsupported OS family: ${::osfamily}")
    }
  }

  $ruby_environment_file = '/etc/profile.d/ruby.sh'
}
