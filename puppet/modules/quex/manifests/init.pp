# == Class: quex
class quex {
    $quex_version = "0.65.10"

    archive { 'quex':
        ensure => present,
        url    => "https://downloads.sourceforge.net/project/quex/DOWNLOAD/quex-${quex_version}.tar.gz",
        target => '/nubots/toolchain/etc/quex',
        src_target => '/nubots/toolchain/src',
        follow_redirects => true,
        strip_components => 2,
        checksum => false
    }
    file { '/nubots/toolchain/include/quex':
        ensure => 'link',
        target => '/nubots/toolchain/etc/quex/quex',
        require => Class['installer::prerequisites'],
    }
    file { 'install-quex-bin':
        path => '/nubots/toolchain/bin/quex',
        ensure => present,
        mode => '755',
        source => 'puppet:///modules/quex/quex',
        require => Class['installer::prerequisites'],
    }
}
