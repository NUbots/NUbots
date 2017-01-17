include apt

# http://www.puppetcookbook.com/posts/set-global-exec-path.html
Exec { path => [ "/bin/", "/sbin/" , "/usr/bin/", "/usr/sbin/" ] }

node nubotsvm {

  # We need dev tools
  class {'dev_tools': }

  # Get and install our toolchain
  $toolchain_version = '1.1.6'
  wget::fetch { 'nubots_deb':
    destination => "/root/nubots-toolchain${toolchain_version}.deb",
    source => "http://nubots.net/debs/nubots-toolchain${toolchain_version}.deb",
    timeout => 0,
  } ->
  package { 'nubots-toolchain':
    provider => 'dpkg',
    ensure => 'latest',
    source => "/root/nubots-toolchain${toolchain_version}.deb",
  }
}

node nubotsvmbuild {
  $archs = {
    'native'   => {'flags'       => ['', ],
                   'params'      => ['-m64', ],
                   'environment' => {'USE_THREAD' => '1', 'BINARY' => '64', 'NUM_THREADS' => '2', 'AUDIO' => 'PORTAUDIO', 'LDFLAGS' => '-m64', },
                  },
    'DarwinOp' => {'flags'       => ['-march=bonnell', '-mtune=bonnell', '-mno-movbe', '-mfxsr', '-mmmx', '-msahf', '-msse', '-msse2', '-msse3', '-mssse3', ],
                   'params'      => ['-m32', '--param l1-cache-size=24', '--param l1-cache-line-size=64', '--param l2-cache-size=512', ],
                   'environment' => {'TARGET' => 'YONAH', 'USE_THREAD' => '1', 'BINARY' => '32', 'NUM_THREADS' => '2', 'AUDIO' => 'PORTAUDIO', 'LDFLAGS' => '-m32', },
                   },
    'NimbroOp' => {'flags'       => ['-march=broadwell', '-mtune=broadwell', '-mabm', '-madx', '-maes', '-mavx', '-mavx2', '-mbmi', '-mbmi2', '-mcx16', '-mf16c', '-mfma', '-mfsgsbase', '-mfxsr', '-mlzcnt', '-mmmx', '-mmovbe', '-mpclmul', '-mpopcnt', '-mprfchw', '-mrdrnd', '-mrdseed', '-msahf', '-msse', '-msse2', '-msse3', '-msse4', '-msse4.1', '-msse4.2', '-mssse3', '-mxsave', '-mxsaveopt', ],
                   'params'      => ['-m64', '--param l1-cache-size=32', '--param l1-cache-line-size=64', '--param l2-cache-size=4096', ],
                   'environment' => {'TARGET' => 'HASWELL', 'USE_THREAD' => '1', 'BINARY' => '64', 'NUM_THREADS' => '2', 'AUDIO' => 'PORTAUDIO', 'LDFLAGS' => '-m64', },
                  },
  }

  # Make sure the necessary installer prerequisites are satisfied.
  class { 'installer::prerequisites' :
    archs => $archs,
  }

  # We need dev tools to use the installer
  class {'dev_tools': } -> Installer <| |>

  # List all of the archives that need to be downloaded along with any other associated parameters (creates, requires, etc).
  $archives = {
    'protobuf'     => {'url'         => 'https://github.com/google/protobuf/releases/download/v3.0.2/protobuf-python-3.0.2.tar.gz',
                       'args'        => { 'native'   => [ '--with-zlib', '--with-protoc=PROTOC_PATH', ],
                                          'DarwinOp' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', '--with-zlib', '--with-protoc=PROTOC_PATH', ],
                                          'NimbroOp' => [ '--with-zlib', '--with-protoc=PROTOC_PATH', ], },
                       'require'     => [ Class['protobuf'], Installer['zlib'], ],
                       'prebuild'    => 'make distclean',
                       'postbuild'   => 'rm PREFIX/lib/libprotoc* && rm PREFIX/bin/protoc',
                       'method'      => 'autotools', },
    'zlib'         => {'url'         => 'http://www.zlib.net/zlib-1.2.10.tar.gz',
                       'creates'     => 'lib/libz.a',
                       'method'      => 'cmake',},
    'bzip2'        => {'url'         => 'https://github.com/Bidski/bzip2/archive/v1.0.6.tar.gz',
                       'creates'     => 'lib/libbz2.so',
                       'method'      => 'make',},
    'xml2'         => {'url'         => 'http://xmlsoft.org/sources/libxml2-2.9.3.tar.gz',
                       'args'        => { 'native'   => [ '--with-zlib=ZLIB_PATH', '--without-python', ],
                                          'DarwinOp' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', '--with-zlib=ZLIB_PATH', '--without-python', ],
                                          'NimbroOp' => [ '--with-zlib=ZLIB_PATH', '--without-python', ], },
                       'method'      => 'autotools',},
    'nuclear'      => {'url'         => 'https://github.com/Fastcode/NUClear/archive/release/1.0.tar.gz',
                       'args'        => { 'native'   => [ '-DBUILD_TESTS=OFF', ],
                                          'DarwinOp' => [ '-DBUILD_TESTS=OFF', ],
                                          'NimbroOp' => [ '-DBUILD_TESTS=OFF', ], },
                       'method'      => 'cmake',},
    # NOTE: OpenBLAS CMake support is experimental and only supports x86 at the moment.
    'openblas'     => {'url'         => 'https://github.com/xianyi/OpenBLAS/archive/v0.2.18.tar.gz',
                       'method'      => 'make',},
    'libsvm'       => {'url'         => 'https://github.com/Bidski/libsvm/archive/v322.tar.gz',
                       'creates'     =>'lib/svm.o',
                       'method'      => 'make', },
    'armadillo'    => {'url'         => 'https://downloads.sourceforge.net/project/arma/armadillo-7.200.2.tar.xz',
                       'method'      => 'cmake',
                       'creates'     => 'lib/libarmadillo.so',
                       'require'     => [ Installer['openblas'], ],},
    'tcmalloc'     => {'url'         => 'https://github.com/gperftools/gperftools/releases/download/gperftools-2.5/gperftools-2.5.tar.gz',
                       'args'        => { 'native'   => [ '--with-tcmalloc-pagesize=64', '--enable-minimal', ],
                                          'DarwinOp' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', '--with-tcmalloc-pagesize=64', '--enable-minimal', ],
                                          'NimbroOp' => [ '--with-tcmalloc-pagesize=64', '--enable-minimal', ], },
                       'creates'     => 'lib/libtcmalloc_minimal.a',
                       'method'      => 'autotools',},
    'yaml-cpp'     => {'url'         => 'https://github.com/jbeder/yaml-cpp/archive/master.tar.gz',
                       'args'        => { 'native'   => [ '-DYAML_CPP_BUILD_CONTRIB=OFF', '-DYAML_CPP_BUILD_TOOLS=OFF', ],
                                          'DarwinOp' => [ '-DYAML_CPP_BUILD_CONTRIB=OFF', '-DYAML_CPP_BUILD_TOOLS=OFF', ],
                                          'NimbroOp' => [ '-DYAML_CPP_BUILD_CONTRIB=OFF', '-DYAML_CPP_BUILD_TOOLS=OFF', ], },
                       'method'      => 'cmake',},
    'fftw3'        => {'url'         => 'http://www.fftw.org/fftw-3.3.4.tar.gz',
                       'args'        => { 'native'   => [ '--disable-fortran', '--enable-shared', ],
                                          'DarwinOp' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', '--disable-fortran', '--enable-shared', ],
                                          'NimbroOp' => [ '--disable-fortran', '--enable-shared', ], },
                       'method'      => 'autotools',},
    'jpeg'         => {'url'         => 'http://downloads.sourceforge.net/project/libjpeg-turbo/1.4.2/libjpeg-turbo-1.4.2.tar.gz',
                       'args'        => { 'native'   => [ 'CCASFLAGS="-f elf64"', ],
                                          'DarwinOp' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', 'CCASFLAGS="-f elf32"', ],
                                          'NimbroOp' => [ 'CCASFLAGS="-f elf64"', ], },
                       'method'      => 'autotools',},
    'cppformat'    => {'url'         => 'https://github.com/cppformat/cppformat/archive/2.0.0.tar.gz',
                       'method'      => 'cmake',},
    'portaudio'    => {'url'         => 'http://www.portaudio.com/archives/pa_stable_v19_20140130.tgz',
                       'args'        => { 'native'   => [ '', ],
                                          'DarwinOp' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', ],
                                          'NimbroOp' => [ '', ], },
                       'method'      => 'autotools',},
    'rtaudio'      => {'url'         => 'http://www.music.mcgill.ca/~gary/rtaudio/release/rtaudio-4.1.1.tar.gz',
                       'args'        => { 'native'   => [ '', ],
                                          'DarwinOp' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', ],
                                          'NimbroOp' => [ '', ], },
                       'method'      => 'autotools',},
    'muparserx'    => {'url'         => 'https://github.com/beltoforion/muparserx/archive/v4.0.4.tar.gz',
                       'method'      => 'cmake',},
    'eigen3'       => {'url'         => 'http://bitbucket.org/eigen/eigen/get/3.2.7.tar.gz',
                       'creates'     => 'include/eigen3/Eigen/Eigen',
                       'method'      => 'cmake',},
    'boost'        => {'url'         => 'http://downloads.sourceforge.net/project/boost/boost/1.59.0/boost_1_59_0.tar.gz',
                       'args'        => { 'native'   => [ 'address-model=64', 'architecture=x86', 'link=static', ],
                                          'DarwinOp' => [ 'address-model=32', 'architecture=x86', 'link=static', ],
                                          'NimbroOp' => [ 'address-model=64', 'architecture=x86', 'link=static', ], },
                       'method'      => 'boost',
                       'creates'     => 'src/boost/build_complete',
                       'postbuild'   => 'touch build_complete',
                       'require'     => [ Installer['zlib'], Installer['bzip2'], ],},
    'mlpack'       => {'url'         => 'https://github.com/mlpack/mlpack/archive/mlpack-2.1.0.tar.gz',
                       'args'        => { 'native'   => [ '-DBUILD_TESTS=OFF', '-DLAPACK_LIBRARY=PREFIX/lib/libopenblas.so', '-DBLAS_LIBRARY=PREFIX/lib/libopenblas.so', ],
                                          'DarwinOp' => [ '-DBUILD_TESTS=OFF', '-DLAPACK_LIBRARY=PREFIX/lib/libopenblas.so', '-DBLAS_LIBRARY=PREFIX/lib/libopenblas.so', ],
                                          'NimbroOp' => [ '-DBUILD_TESTS=OFF', '-DLAPACK_LIBRARY=PREFIX/lib/libopenblas.so', '-DBLAS_LIBRARY=PREFIX/lib/libopenblas.so', ], },
                       'require'     => [ Installer['armadillo'], Installer['boost'], Installer['xml2'], ],
                       'creates'     => 'lib/libmlpack.so',
                       'method'      => 'cmake',},
    'espeak'       => {'url'         => 'https://github.com/Bidski/espeak/archive/v1.48.04.tar.gz',
                       'src_dir'     => 'src',
                       'prebuild'    => 'cp portaudio19.h portaudio.h',
                       'method'      => 'make',
                       'require'     => [ Installer['portaudio'], ],},
    'raw1394'      => {'url'         => 'http://downloads.sourceforge.net/project/libraw1394/libraw1394/libraw1394-2.0.5.tar.gz',
                       'args'        => { 'native'   => [ '', ],
                                          'DarwinOp' => [ '', ],
                                          'NimbroOp' => [ '', ], },
                       'method'      => 'autotools',},
    'dc1394'       => {'url'         => 'http://downloads.sourceforge.net/project/libdc1394/libdc1394-2/2.2.4/libdc1394-2.2.4.tar.gz',
                       'args'        => { 'native'   => [ '--disable-doxygen-doc', '--disable-examples', ],
                                          'DarwinOp' => [ '--disable-doxygen-doc', '--disable-examples', ],
                                          'NimbroOp' => [ '--disable-doxygen-doc', '--disable-examples', ], },
                       'method'      => 'autotools',
                       'require'     => [ Installer['raw1394'], ],},
    'pybind11'     => {'url'         => 'https://github.com/pybind/pybind11/archive/v2.0.1.tar.gz',
                       'args'        => { 'native'   => [ '-DPYBIND11_TEST=OFF', '-DPYTHON_EXECUTABLE=PREFIX/pypy/bin/pypy' ],
                                          'DarwinOp' => [ '-DPYBIND11_TEST=OFF', '-DPYTHON_EXECUTABLE=PREFIX/pypy/bin/pypy' ],
                                          'NimbroOp' => [ '-DPYBIND11_TEST=OFF', '-DPYTHON_EXECUTABLE=PREFIX/pypy/bin/pypy' ], },
                       'creates'     => 'lib/libpybind11.so',
                       'require'     => [ Installer['eigen3'], Exec['PyPy_native_Files'], Exec['PyPy_NimbroOp_Files'], Exec['PyPy_DarwinOp_Files'], ],
                       'method'      => 'cmake',},
    'fswatch'      => {'url'         => 'https://github.com/emcrisostomo/fswatch/archive/1.9.3.tar.gz',
                       'args'        => { 'native'   => [ '', ],
                                          'DarwinOp' => [ '--host=i686-linux-gnu', '--build=x86_64-unknown-linux-gnu', ],
                                          'NimbroOp' => [ '', ], },
                       'method'      => 'autotools', },
  }

  # Download each archive and spawn Installers for each one.
  $archives.each |String $archive,
                  Struct[{'url' => String,
                          Optional['creates'] => String,
                          Optional['args'] => Hash,
                          Optional['require'] => Tuple[Any, 1, default],
                          'method' => String,
                          Optional['src_dir'] => String,
                          Optional['prebuild'] => String,
                          Optional['postbuild'] => String}] $params| {

        $extension = $params['url'] ? {
          /.*\.zip/       => 'zip',
          /.*\.tgz/       => 'tgz',
          /.*\.tar\.gz/   => 'tar.gz',
          /.*\.txz/       => 'txz',
          /.*\.tar\.xz/   => 'tar.xz',
          /.*\.tbz/       => 'tbz',
          /.*\.tbz2/      => 'tbz2',
          /.*\.tar\.bz2/  => 'tar.bz2',
          /.*\.h/         => 'h',
          /.*\.hpp/       => 'hpp',
          default         => 'UNKNOWN',
        }

        archive { "${archive}":
          url              => $params['url'],
          target           => "/nubots/toolchain/src/${archive}",
          src_target       => "/nubots/toolchain/src",
          purge_target     => true,
          checksum         => false,
          follow_redirects => true,
          timeout          => 0,
          extension        => $extension,
          strip_components => 1,
          root_dir         => '.',
          require          => [ Class['installer::prerequisites'], Class['dev_tools'], ],
        }
        installer { "${archive}":
          archs       => $archs,
          creates     => $params['creates'],
          require     => delete_undef_values(flatten([ Archive["${archive}"], $params['require'], Class['installer::prerequisites'], Class['dev_tools'], ])),
          args        => $params['args'],
          src_dir     => $params['src_dir'],
          prebuild    => $params['prebuild'],
          postbuild   => $params['postbuild'],
          method      => $params['method'],
          extension   => $extension,
        }
  }

  # Install quex.
  class { 'quex': }

  # Install protobuf.
  class { 'protobuf': }

  # Install catch.
  installer { 'catch':
    url       => 'https://raw.githubusercontent.com/philsquared/Catch/master/single_include/catch.hpp',
    archs     => $archs,
    extension => 'hpp',
    method    => 'wget',
  }

  # Perform any complicated postbuild instructions here.
  $archs.each |String $arch, Hash $params| {
    # Update the armadillo config header file for all archs.
    file { "armadillo_${arch}_config":
      path    => "/nubots/toolchain/${arch}/include/armadillo_bits/config.hpp",
      source  => 'puppet:///modules/files/nubots/toolchain/include/armadillo_bits/config.hpp',
      ensure  => present,
      require => [ Installer['armadillo'], ],
    }
  }

  archive { "Spinnaker_amd64":
    url              => "http://nubots.net/tarballs/spinnaker_1_0_0_295_amd64.tar.gz",
    target           => "/nubots/toolchain/NimbroOp/src/Spinnaker",
    src_target       => "/nubots/toolchain/NimbroOp/src",
    purge_target     => true,
    checksum         => false,
    follow_redirects => true,
    timeout          => 0,
    extension        => "tar.gz",
    strip_components => 1,
    root_dir         => '.',
    require          => [ Class['installer::prerequisites'], Class['dev_tools'], ],
  }
  archive { "Spinnaker_i386":
    url              => "http://nubots.net/tarballs/spinnaker_1_0_0_295_i386.tar.gz",
    target           => "/nubots/toolchain/DarwinOp/src/Spinnaker",
    src_target       => "/nubots/toolchain/DarwinOp/src",
    purge_target     => true,
    checksum         => false,
    follow_redirects => true,
    timeout          => 0,
    extension        => "tar.gz",
    strip_components => 1,
    root_dir         => '.',
    require          => [ Class['installer::prerequisites'], Class['dev_tools'], ],
  }
  exec { "Spinnaker_amd64_Files":
    creates  => "/nubots/toolchain/NimbroOp/include/Spinnaker.h",
    command  => "cd include && cp -r ./* /nubots/toolchain/NimbroOp/include/ && cd .. &&
                 cd lib && 
                 cp libGCBase_gcc540_v3_0.so* /nubots/toolchain/NimbroOp/lib/ &&
                 cp libGenApi_gcc540_v3_0.so* /nubots/toolchain/NimbroOp/lib/ &&
                 cp libLog_gcc540_v3_0.so* /nubots/toolchain/NimbroOp/lib/ &&
                 cp libMathParser_gcc540_v3_0.so* /nubots/toolchain/NimbroOp/lib/ &&
                 cp libNodeMapData_gcc540_v3_0.so* /nubots/toolchain/NimbroOp/lib/ &&
                 cp libptgreyvideoencoder.so* /nubots/toolchain/NimbroOp/lib/ &&
                 cp libSpinnaker.so* /nubots/toolchain/NimbroOp/lib/ &&
                 cp libXmlParser_gcc540_v3_0.so* /nubots/toolchain/NimbroOp/lib/ &&
                 cd ..",
    cwd      => "/nubots/toolchain/NimbroOp/src/Spinnaker",
    path     =>  [ "/nubots/toolchain/NimbroOp/bin", "/nubots/toolchain/bin",
                   '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
    timeout  => 0,
    provider => 'shell',
    require  => [ Archive["Spinnaker_amd64"], ],
    before   => Class['toolchain_deb'],
  }
  exec { "Spinnaker_i386_Files":
    creates  => "/nubots/toolchain/NimbroOp/include/Spinnaker.h",
    command  => "cd include && cp -r ./* /nubots/toolchain/NimbroOp/include/ && cd .. &&
                 cd lib && 
                 cp libGCBase_gcc540_v3_0.so* /nubots/toolchain/NimbroOp/lib/ &&
                 cp libGenApi_gcc540_v3_0.so* /nubots/toolchain/NimbroOp/lib/ &&
                 cp libLog_gcc540_v3_0.so* /nubots/toolchain/NimbroOp/lib/ &&
                 cp libMathParser_gcc540_v3_0.so* /nubots/toolchain/NimbroOp/lib/ &&
                 cp libNodeMapData_gcc540_v3_0.so* /nubots/toolchain/NimbroOp/lib/ &&
                 cp libptgreyvideoencoder.so* /nubots/toolchain/NimbroOp/lib/ &&
                 cp libSpinnaker.so* /nubots/toolchain/NimbroOp/lib/ &&
                 cp libXmlParser_gcc540_v3_0.so* /nubots/toolchain/NimbroOp/lib/ &&
                 cd ..",
    cwd      => "/nubots/toolchain/DarwinOp/src/Spinnaker",
    path     =>  [ "/nubots/toolchain/DarwinOp/bin", "/nubots/toolchain/bin",
                   '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
    timeout  => 0,
    provider => 'shell',
    require  => [ Archive["Spinnaker_i386"], ],
    before   => Class['toolchain_deb'],
  }

  archive { "PyPy_native":
    url              => "https://bitbucket.org/squeaky/portable-pypy/downloads/pypy3.3-5.5-alpha-20161013-linux_x86_64-portable.tar.bz2",
    target           => "/nubots/toolchain/native/src/pypy",
    src_target       => "/nubots/toolchain/native/src",
    purge_target     => false,
    checksum         => false,
    follow_redirects => true,
    timeout          => 0,
    extension        => "tar.bz2",
    strip_components => 1,
    root_dir         => '.',
    require          => [ Class['installer::prerequisites'], Class['dev_tools'], ],
  }
  archive { "PyPy_NimbroOp":
    url              => "https://bitbucket.org/squeaky/portable-pypy/downloads/pypy3.3-5.5-alpha-20161013-linux_x86_64-portable.tar.bz2",
    target           => "/nubots/toolchain/NimbroOp/src/pypy",
    src_target       => "/nubots/toolchain/NimbroOp/src",
    purge_target     => false,
    checksum         => false,
    follow_redirects => true,
    timeout          => 0,
    extension        => "tar.bz2",
    strip_components => 1,
    root_dir         => '.',
    require          => [ Class['installer::prerequisites'], Class['dev_tools'], ],
  }
  archive { "PyPy_DarwinOp":
    url              => "https://bitbucket.org/squeaky/portable-pypy/downloads/pypy3.3-5.5-alpha-20161014-linux_i686-portable.tar.bz2",
    target           => "/nubots/toolchain/DarwinOp/src/pypy",
    src_target       => "/nubots/toolchain/DarwinOp/src",
    purge_target     => false,
    checksum         => false,
    follow_redirects => true,
    timeout          => 0,
    extension        => "tar.bz2",
    strip_components => 1,
    root_dir         => '.',
    require          => [ Class['installer::prerequisites'], Class['dev_tools'], ],
  }
  exec { "PyPy_NimbroOp_Files":
    creates     => "/nubots/toolchain/NimbroOp/pypy/bin/pypy",
    command     => "./bin/virtualenv-pypy --always-copy /nubots/toolchain/NimbroOp/pypy &&
                    /nubots/toolchain/NimbroOp/pypy/bin/pip3 install pyparsing &&
                    /nubots/toolchain/NimbroOp/pypy/bin/pip3 install pydotplus &&
                    /nubots/toolchain/NimbroOp/pypy/bin/pip3 install pygments &&
                    /nubots/toolchain/NimbroOp/pypy/bin/pip3 install termcolor &&
                    /nubots/toolchain/NimbroOp/pypy/bin/pip3 install mmh3 &&
                    /nubots/toolchain/NimbroOp/pypy/bin/pip3 install protobuf",
    cwd         => "/nubots/toolchain/NimbroOp/src/pypy",
    path        =>  [ "/nubots/toolchain/NimbroOp/bin", "/nubots/toolchain/bin",
                      '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
    environment => ["LD_LIBRARY_PATH=\"/nubots/toolchain/NimbroOp/lib:/nubots/toolchain/lib\""],
    timeout     => 0,
    provider    => 'shell',
    require     => [ Archive["PyPy_NimbroOp"], Installer['bzip2'], ],
    before      => Class['toolchain_deb'],
  }
  exec { "PyPy_native_Files":
    creates     => "/nubots/toolchain/native/pypy/bin/pypy",
    command     => "./bin/virtualenv-pypy --always-copy /nubots/toolchain/native/pypy &&
                    /nubots/toolchain/native/pypy/bin/pip3 install pyparsing &&
                    /nubots/toolchain/native/pypy/bin/pip3 install pydotplus &&
                    /nubots/toolchain/native/pypy/bin/pip3 install pygments &&
                    /nubots/toolchain/native/pypy/bin/pip3 install termcolor &&
                    /nubots/toolchain/native/pypy/bin/pip3 install mmh3 &&
                    /nubots/toolchain/native/pypy/bin/pip3 install protobuf",
    cwd         => "/nubots/toolchain/native/src/pypy",
    path        =>  [ "/nubots/toolchain/native/bin", "/nubots/toolchain/bin",
                      '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
    environment => ["LD_LIBRARY_PATH=\"/nubots/toolchain/native/lib:/nubots/toolchain/lib\""],
    timeout     => 0,
    provider    => 'shell',
    require     => [ Archive["PyPy_native"], Installer['bzip2'], ],
    before      => Class['toolchain_deb'],
  }
  exec { "PyPy_DarwinOp_Files":
    creates     => "/nubots/toolchain/DarwinOp/pypy/bin/pypy",
    command     => "LD_LIBRARY_PATH=\"/nubots/toolchain/DarwinOp/lib:/nubots/toolchain/lib\" ./bin/virtualenv-pypy --always-copy /nubots/toolchain/DarwinOp/pypy &&
                    LD_LIBRARY_PATH=\"/nubots/toolchain/DarwinOp/lib:/nubots/toolchain/lib\" /nubots/toolchain/DarwinOp/pypy/bin/pip3 install pyparsing &&
                    LD_LIBRARY_PATH=\"/nubots/toolchain/DarwinOp/lib:/nubots/toolchain/lib\" /nubots/toolchain/DarwinOp/pypy/bin/pip3 install pydotplus &&
                    LD_LIBRARY_PATH=\"/nubots/toolchain/DarwinOp/lib:/nubots/toolchain/lib\" /nubots/toolchain/DarwinOp/pypy/bin/pip3 install pygments &&
                    LD_LIBRARY_PATH=\"/nubots/toolchain/DarwinOp/lib:/nubots/toolchain/lib\" /nubots/toolchain/DarwinOp/pypy/bin/pip3 install termcolor &&
                    LD_LIBRARY_PATH=\"/nubots/toolchain/DarwinOp/lib:/nubots/toolchain/lib\" /nubots/toolchain/DarwinOp/pypy/bin/pip3 install mmh3 &&
                    LD_LIBRARY_PATH=\"/nubots/toolchain/DarwinOp/lib:/nubots/toolchain/lib\" /nubots/toolchain/DarwinOp/pypy/bin/pip3 install protobuf",
    cwd         => "/nubots/toolchain/DarwinOp/src/pypy",
    path        =>  [ "/nubots/toolchain/DarwinOp/bin", "/nubots/toolchain/bin",
                      '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
    environment => ["LD_LIBRARY_PATH=\"/nubots/toolchain/DarwinOp/lib:/nubots/toolchain/lib\""],
    timeout     => 0,
    provider    => 'shell',
    require     => [ Archive["PyPy_DarwinOp"], Installer['bzip2'], ],
    before      => Class['toolchain_deb'],
  }

  # After we have installed, create the CMake toolchain files and then build our deb.
  Installer <| |> ~> class { 'toolchain_deb': }

  $archs.each |String $arch, Hash $params| {
    # Create CMake toolchain files.
    $prefix          = '/nubots/toolchain'
    $compile_options = join(prefix(suffix($params['flags'], ')'), 'add_compile_options('), "\n")
    $compile_params  = join($params['params'], " ")

    file { "${arch}.cmake":
      content =>
"set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_C_COMPILER /usr/bin/gcc)
set(CMAKE_CXX_COMPILER /usr/bin/g++)

set(CMAKE_FIND_ROOT_PATH \"${prefix}/${arch}\"
       \"${prefix}\"
       \"/usr/local\"
       \"/usr\")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

${compile_options}

include_directories(SYSTEM \"${prefix}/${arch}/include\")
include_directories(SYSTEM \"${prefix}/include\")

set(CMAKE_C_FLAGS \"\${CMAKE_C_FLAGS} ${compile_params}\" CACHE STRING \"\" FORCE)
set(CMAKE_CXX_FLAGS \"\${CMAKE_CXX_FLAGS} ${compile_params}\" CACHE STRING \"\" FORCE)

set(PLATFORM \"${arch}\" CACHE STRING \"The platform to build for.\" FORCE)

set(PYTHON_EXECUTABLE \"${prefix}/${arch}/pypy/bin/pypy\" CACHE STRING \"Path to the python interpreter to use.\" FORCE)
",
      ensure  => present,
      path    => "${prefix}/${arch}.cmake",
      before  => Class['toolchain_deb'],
    }
  }
}
