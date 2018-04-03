# == Class: python
class python {
    # Build python
    archive { 'python-native':
      url              => 'https://www.python.org/ftp/python/3.6.3/Python-3.6.3.tar.xz',
      target           => '/nubots/toolchain/src/python',
      src_target       => '/nubots/toolchain/src',
      purge_target     => true,
      checksum         => false,
      follow_redirects => true,
      timeout          => 0,
      extension        => 'tar.xz',
      strip_components => 1,
      root_dir         => '.',
      require          => [ Class['installer::prerequisites'], Class['build_tools'], ],
    }

    exec { 'autotools_python':
      creates     => '/nubots/toolchain/bin/python3.6',
      command     => "./configure CFLAGS=\"-fPIC -Os\" CXXFLAGS=\"-fPIC -Os\" --prefix=\"/nubots/toolchain\" --enable-shared &&
                      make -j\$(nproc) &&
                      make install",
      cwd         => '/nubots/toolchain/src/python',
      path        =>  [ '/nubots/toolchain/bin', '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
      timeout     => 0,
      provider    => 'shell',
      require     => [ Archive['python-native'], ],
    }

    $ldflags = "LDFLAGS=-L/nubots/toolchain/lib"
    $pkgconfig = "PKG_CONFIG_PATH=/nubots/toolchain/lib/pkgconfig:/nubots/toolchain/share/pkgconfig"
    $ltsyslibpath = "LT_SYS_LIBRARY_PATH=/nubots/toolchain/lib"
    $ldlibrarypath = "LD_LIBRARY_PATH=/nubots/toolchain/lib"
    $aclocalpath = "ACLOCAL_PATH=/nubots/toolchain/share/aclocal"

    $env = ['CC=/usr/bin/gcc', 'CXX=/usr/bin/g++', $ldflags, $pkgconfig, $ltsyslibpath, $ldlibrarypath, $aclocalpath, ]

    # We need to match the protobuf version with the one we install in the toolchain.
    exec {'install_python3_packages':
      command => "/nubots/toolchain/bin/pip3.6 install pyparsing &&
                  /nubots/toolchain/bin/pip3.6 install pydotplus &&
                  /nubots/toolchain/bin/pip3.6 install pygments &&
                  /nubots/toolchain/bin/pip3.6 install termcolor &&
                  /nubots/toolchain/bin/pip3.6 install protobuf==3.5.0.post1 &&
                  /nubots/toolchain/bin/pip3.6 install numpy &&
                  /nubots/toolchain/bin/pip3.6 install wheel &&
                  /nubots/toolchain/bin/pip3.6 install PyYAML &&
                  /nubots/toolchain/bin/pip3.6 install yapf &&
                  /nubots/toolchain/bin/pip3.6 install xxhash &&
                  /nubots/toolchain/bin/pip3.6 install tensorflow &&
                  /nubots/toolchain/bin/pip3.6 install scikit-image &&
                  /nubots/toolchain/bin/pip3.6 install scipy &&
                  touch /nubots/toolchain/src/pip_packages_complete",
      creates     => "/nubots/toolchain/src/pip_packages_complete",
      path        =>  [ '/nubots/toolchain/bin', '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
      environment => $env,
      timeout     => 0,
      provider    => 'shell',
      require => [ Exec['autotools_python'], ],
    }
}
