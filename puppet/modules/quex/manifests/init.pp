# == Class: quex
class quex {
    $quex_version = "0.65.4"
    archive { 'quex':
        ensure => present,
        url    => "https://downloads.sourceforge.net/project/quex/DOWNLOAD/quex-${quex_version}.tar.gz",
        target => '/usr/local/etc',
        strip_components => 2,
        checksum => false
    } ->
    file { '/usr/local/include/quex':
        ensure => 'link',
        target => '/usr/local/etc/quex/quex',
    } ->
    file { 'install-quex-bin':
        path => '/usr/local/bin/quex',
        ensure => present,
        mode => '755',
        source => 'puppet:///modules/quex/quex',
    }
}
