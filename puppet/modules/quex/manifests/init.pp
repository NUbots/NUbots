# == Class: quex
class quex {
    file { 'install-quex-script':
        path => '/tmp/installquex.sh',
        ensure => present,
        source => 'puppet:///modules/quex/installquex.sh',
    } ->
    exec { 'install-quex':
        command => 'bash /tmp/installquex.sh',
        path => $path,
    }
}
