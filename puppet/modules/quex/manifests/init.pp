# == Class: quex
class quex {

    archive { 'quex':
        ensure => present,
        url    => "https://downloads.sourceforge.net/project/quex/HISTORY/0.65.2/quex-0.65.2.tar.gz",
        target => '/nubots/toolchain/etc/quex',
        src_target => '/nubots/toolchain/src',
        follow_redirects => true,
        strip_components => 2,
        checksum => false,
        require => Class['installer::prerequisites'],
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

