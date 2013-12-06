node npvagrant {
  # Perform a single `apt-get update` before installing ANY packages.
  exec { "initial_apt_update":
    command => "/usr/bin/apt-get update"
  } -> Package <| |>

  # define variables for this node
  $username = 'vagrant'

  class { 'nuclearport::build_dep':
    username => $username,
  }

  class { 'vim': 
    username => $username,
  }
}
