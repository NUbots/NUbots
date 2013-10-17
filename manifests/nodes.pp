class vim {
  package { 'vim':
    ensure => latest
  } ->
  package { 'vim-puppet':
    ensure => latest,
  } ->
  file { ['/home/mitchell/.vim', '/home/mitchell/.vim/plugin']:
    ensure => directory,
  } ->
  file { '/home/mitchell/.vim/plugin/puppet.vim':
    ensure => link,
    target => '/usr/share/vim/addons/syntax/puppet.vim',
  }
}


node default {
  package { 'git':
    ensure => latest,
  }

  package { 'openssh-server':
    ensure => latest,
  }

  include vim  
}
