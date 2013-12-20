# == Class: catch
#
# Full description of class catch here.
#
# === Parameters
#
# Document parameters here.
#
# [*sample_parameter*]
#   Explanation of what this parameter affects and what it defaults to.
#   e.g. "Specify one or more upstream ntp servers as an array."
#
# === Variables
#
# Here you should define a list of variables that this module would require.
#
# [*sample_variable*]
#   Explanation of how this variable affects the funtion of this class and if
#   it has a default. e.g. "The parameter enc_ntp_servers must be set by the
#   External Node Classifier as a comma separated list of hostnames." (Note,
#   global variables should be avoided in favor of class parameters as
#   of Puppet 2.6.)
#
# === Examples
#
#  class { catch:
#    servers => [ 'pool.ntp.org', 'ntp.local.company.com' ],
#  }
#
# === Authors
#
# Author Name <author@domain.com>
#
# === Copyright
#
# Copyright 2013 Your name here, unless otherwise noted.
#
# EDIT: Changed fetch command to download from an earlier commit (Build 14).
class catch {
  wget::fetch { 'catch.hpp':
    destination => '/usr/local/include/catch.hpp',
    source => 'https://raw.github.com/philsquared/Catch/5ecb72b9bb65cd8fed2aec4da23a3bc21bbccd74/single_include/catch.hpp',
  }
}
