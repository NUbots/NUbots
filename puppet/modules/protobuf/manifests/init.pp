# == Class: protobuf
class protobuf {
    # Build ELLCC
    archive { 'protobuf-native':
      url              => 'https://github.com/google/protobuf/releases/download/v3.0.0-beta-3/protobuf-python-3.0.0-beta-3.tar.gz',
      target           => '/nubots/toolchain/src/protobuf',
      src_target       => '/nubots/toolchain/src',
      purge_target     => true,
      checksum         => false,
      follow_redirects => true,
      timeout          => 0,
      extension        => 'tar.gz',
      strip_components => 1,
      root_dir         => '.',
      require          => [ Class['installer::prerequisites'], Class['dev_tools'], ],
    }

    exec { 'autotools_protobuf':
      creates     => '/nubots/toolchain/bin/protoc',
      command     => "cp protobuf-native.tar.gz protobuf.tar.gz && 
                      cd protobuf &&
                      ./configure --prefix=\"/nubots/toolchain\" --with-zlib &&
                      make -j\$(nproc) &&
                      make install",
      cwd         => '/nubots/toolchain/src',
      path        =>  [ '/nubots/toolchain/bin', '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
      timeout     => 0,
      provider    => 'shell',
      require     => [ Archive['protobuf-native'], ],
    }
}

