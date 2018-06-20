# == Class: protobuf
class protobuf {
    # Build protobuf
    archive { 'protobuf-native':
      url              => 'https://github.com/google/protobuf/releases/download/v3.5.0/protobuf-cpp-3.5.0.tar.gz',
      target           => '/nubots/toolchain/src/protobuf',
      src_target       => '/nubots/toolchain/src',
      purge_target     => true,
      checksum         => false,
      follow_redirects => true,
      timeout          => 0,
      extension        => 'tar.gz',
      strip_components => 1,
      root_dir         => '.',
      require          => [ Class['installer::prerequisites'], Class['build_tools'], ],
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
