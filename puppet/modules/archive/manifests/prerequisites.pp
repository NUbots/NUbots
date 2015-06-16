# == Class: prerequisites
#
# Ensure any required dependencies for archive download and extraction are present.
#
# Parameters:
#
# None
#
class archive::prerequisites {

  # list of packages needed for download and extraction
  $packages = [ 'curl', 'unzip', 'tar', ]

  # install additional packages if missing
  package { $packages:
    ensure => installed,
  }
}
