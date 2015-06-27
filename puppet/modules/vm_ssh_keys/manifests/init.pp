# == Class: vm_ssh_keys
class vm_ssh_keys {
    file { 'vm_private_key':
        path => '/home/vagrant/.ssh/id_rsa',
        ensure => present,
        source => 'puppet:///modules/vm_ssh_keys/id_rsa',
        owner => 'vagrant',
        mode => '600',
    }

    file { 'vm_public_key':
        path => '/home/vagrant/.ssh/id_rsa.pub',
        ensure => present,
        source => 'puppet:///modules/vm_ssh_keys/id_rsa.pub',
        owner => 'vagrant',
    }
}
