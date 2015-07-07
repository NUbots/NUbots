# == Class: quex
class quex {
    $quex_version = "0.65.4"
    archive { 'quex':
        ensure => present,
        url    => "https://downloads.sourceforge.net/project/quex/DOWNLOAD/quex-${quex_version}.tar.gz",
        target => '/nubots/toolchain/src',
        strip_components => 2,
        checksum => false
    } ->
    file { '/nubots/toolchain/include/quex':
        ensure => 'link',
        target => '/nubots/toolchain/etc/quex/quex',
    } ->
    file { 'install-quex-bin':
        path => '/nubots/toolchain/bin/quex',
        ensure => present,
        mode => '755',
        source => 'puppet:///modules/quex/quex',
    }
}
